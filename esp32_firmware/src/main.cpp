#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "FloodFill.h"

static SensorArray sensors;
static MotorController motors;
static FloodFill solver;

static int currentX = START_X;
static int currentY = START_Y;
static Direction currentHeading = NORTH;

void toneBeep(int freq, int durationMs) {
    if (PIN_BUZZER >= 0) {
        tone(PIN_BUZZER, freq, durationMs);
    }
}

void turnTo(Direction targetDir) {
    TurnCommand turnCmd = FloodFill::getTurnCommand(currentHeading, targetDir);
    switch (turnCmd) {
        case TURN_LEFT:
            motors.turnLeft();
            break;
        case TURN_RIGHT:
            motors.turnRight();
            break;
        case TURN_AROUND:
            motors.turnAround();
            break;
        case TURN_NONE:
        default:
            break;
    }
    currentHeading = targetDir;
}

void waitForStartButton() {
    Serial.println(F("[Micromouse] Robot Ready! Place at (0,0) facing North."));
    Serial.println(F("[Micromouse] Press physical START button to launch run..."));

    if (PIN_STATUS_LED >= 0) pinMode(PIN_STATUS_LED, OUTPUT);
    if (PIN_START_BUTTON >= 0) pinMode(PIN_START_BUTTON, INPUT_PULLUP);

    // Fast blink while waiting for user button
    while (true) {
        if (PIN_START_BUTTON >= 0 && digitalRead(PIN_START_BUTTON) == LOW) {
            delay(50); // Debounce
            if (digitalRead(PIN_START_BUTTON) == LOW) break;
        }
        if (PIN_STATUS_LED >= 0) {
            digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
        }
        delay(100);
    }

    // 3-second countdown with beeps
    Serial.println(F("[Micromouse] Start button pressed! Starting countdown..."));
    for (int i = 3; i > 0; --i) {
        Serial.printf("[Micromouse] Launching in %d...\n", i);
        toneBeep(1000, 150);
        if (PIN_STATUS_LED >= 0) digitalWrite(PIN_STATUS_LED, HIGH);
        delay(500);
        if (PIN_STATUS_LED >= 0) digitalWrite(PIN_STATUS_LED, LOW);
        delay(500);
    }
    toneBeep(2000, 400); // High pitch GO beep
    if (PIN_STATUS_LED >= 0) digitalWrite(PIN_STATUS_LED, HIGH);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("=================================================="));
    Serial.println(F("  AUTONOMOUS 10x10 MICROMOUSE ESP32 FIRMWARE     "));
    Serial.println(F("  Championship Flood Fill + Turn-Weighted Sprint "));
    Serial.println(F("=================================================="));

    // Initialize hardware drivers
    sensors.init();
    motors.init();
    solver.init(DEFAULT_MAZE_WIDTH, DEFAULT_MAZE_HEIGHT);

    // Wait for physical user release at starting cell (0,0)
    waitForStartButton();

    // ========================================================================
    // PHASE 1: Search Run (Start -> 10x10 Center Goals: (4,4), (4,5), (5,4), (5,5))
    // ========================================================================
    Serial.println(F("=== PHASE 1: SEARCH RUN (Mapping Maze) ==="));
    int searchSteps = 0;

    while (!solver.isGoal(currentX, currentY)) {
        if (sensors.hasTactileCollision()) {
            Serial.println(F("[EMERGENCY] Bumper Collision! Motors stopped."));
            motors.stop();
            return;
        }

        solver.markVisited(currentX, currentY);

        // 1. Poll Sensors (2x Front ToF + 2x Side IR)
        sensors.update();

        // 2. Discover Walls & Re-flood
        solver.updateWalls(
            currentX, currentY, currentHeading,
            sensors.hasWallFront(), sensors.hasWallLeft(), sensors.hasWallRight()
        );

        if (solver.isGoal(currentX, currentY)) break;

        // 3. Decide Next Move & Execute Turn
        Direction nextDir = solver.getNextMove(currentX, currentY, currentHeading);
        turnTo(nextDir);

        // 4. Drive forward 1 cell
        motors.moveForward(1, SPEED_SEARCH_PWM);
        CellCoord nextCoord = FloodFill::advanceCoord(currentX, currentY, currentHeading);
        currentX = nextCoord.x;
        currentY = nextCoord.y;
        searchSteps++;

        Serial.printf("[Search] Step %d -> at (%d, %d)\n", searchSteps, currentX, currentY);
    }

    Serial.printf("[Search] Goal Reached at (%d, %d) in %d steps!\n", currentX, currentY, searchSteps);
    toneBeep(1500, 300);
    delay(1000); // 1-second pause at center

    // ========================================================================
    // PHASE 2: Return Run (Center Goal -> Start Cell (0,0))
    // ========================================================================
    Serial.println(F("=== PHASE 2: RETURN RUN (Heading to (0,0)) ==="));
    solver.setGoalsStart();
    int returnSteps = 0;

    while (currentX != START_X || currentY != START_Y) {
        if (sensors.hasTactileCollision()) {
            motors.stop();
            return;
        }

        solver.markVisited(currentX, currentY);
        sensors.update();
        solver.updateWalls(
            currentX, currentY, currentHeading,
            sensors.hasWallFront(), sensors.hasWallLeft(), sensors.hasWallRight()
        );

        if (currentX == START_X && currentY == START_Y) break;

        Direction nextDir = solver.getNextMove(currentX, currentY, currentHeading);
        turnTo(nextDir);

        motors.moveForward(1, SPEED_SEARCH_PWM);
        CellCoord nextCoord = FloodFill::advanceCoord(currentX, currentY, currentHeading);
        currentX = nextCoord.x;
        currentY = nextCoord.y;
        returnSteps++;
    }

    // Orient facing North at start
    turnTo(NORTH);
    Serial.printf("[Return] Returned to (0,0) in %d steps. Ready for sprint!\n", returnSteps);
    toneBeep(1800, 300);
    delay(2000); // 2-second setup pause before Speed Run

    // ========================================================================
    // PHASE 3: Championship Speed Run (Turn-Weighted Optimal Sprint)
    // ========================================================================
    Serial.println(F("=== PHASE 3: CHAMPIONSHIP SPEED RUN ==="));
    solver.setGoalsCenter();

    int pathSteps = solver.computeSpeedPath(START_X, START_Y, NORTH);
    const PathStep* speedPath = solver.getSpeedPath();

    Serial.printf("[SpeedRun] Optimal path planned: %d macro-bursts.\n", pathSteps);

    int totalCells = 0;
    for (int i = 0; i < pathSteps; ++i) {
        if (sensors.hasTactileCollision()) {
            motors.stop();
            return;
        }

        const PathStep& step = speedPath[i];
        turnTo(step.dir);

        // High-speed multi-cell burst!
        Serial.printf("[SpeedRun] Sprinting %d cell(s) in direction %d\n", step.straightCells, step.dir);
        motors.moveForward(step.straightCells, SPEED_SPRINT_PWM);

        currentX += DIR_DX[step.dir] * step.straightCells;
        currentY += DIR_DY[step.dir] * step.straightCells;
        totalCells += step.straightCells;
    }

    motors.stop();
    Serial.printf("[SpeedRun] 🏁 VICTORY! Reached goal in %d cells across %d bursts!\n", totalCells, pathSteps);

    // Victory fanfare
    toneBeep(1200, 150);
    delay(50);
    toneBeep(1500, 150);
    delay(50);
    toneBeep(2000, 400);
}

void loop() {
    // Robot parked at center goal after completing run
    delay(1000);
}
