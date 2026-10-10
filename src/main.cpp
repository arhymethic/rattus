#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "FloodFill.h"

#include <iostream>
#include <string>

#ifdef SIMULATION
#include "API.h"

void logMessage(const std::string& msg) {
    std::cerr << "[Micromouse] " << msg << std::endl;
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    logMessage("Starting IEEE UCI Flood Fill Autonomous Micromouse (3-Phase Championship Engine)...");

    // Query MMS for actual maze dimensions (handles 10x10, 16x16, etc.)
    int mazeW = API::mazeWidth();
    int mazeH = API::mazeHeight();
    if (mazeW <= 0 || mazeW > MAX_MAZE_DIM) mazeW = DEFAULT_MAZE_WIDTH;
    if (mazeH <= 0 || mazeH > MAX_MAZE_DIM) mazeH = DEFAULT_MAZE_HEIGHT;

    logMessage("Configured Maze Grid: " + std::to_string(mazeW) + "x" + std::to_string(mazeH));

    SensorArray sensors;
    MotorController motors;
    FloodFill solver;

    sensors.init();
    motors.init();
    solver.init(mazeW, mazeH);
    solver.initMMSVisuals();

    int currentX = START_X;
    int currentY = START_Y;
    Direction currentHeading = NORTH;

    auto checkReset = [&]() -> bool {
        if (API::wasReset()) {
            API::ackReset();
            currentX = START_X;
            currentY = START_Y;
            currentHeading = NORTH;
            logMessage("MMS Reset acknowledged. Mouse repositioned at (0,0) North.");
            return true;
        }
        return false;
    };

    auto turnTo = [&](Direction targetDir) {
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
    };

    // ========================================================================
    // PHASE 1: Search Run (Start (0,0) -> Center Goal)
    // Maps maze walls using IEEE UCI flood fill BFS gradient.
    // ========================================================================
    logMessage("=== Phase 1: Search Run (Mapping Maze to Center Goal) ===");
    int searchSteps = 0;

    while (!solver.isGoal(currentX, currentY)) {
        if (checkReset()) break;

        solver.markVisited(currentX, currentY);
        API::setColor(currentX, currentY, 'C'); // Cyan for search run path

        // Poll sensors (2x Front ToF + 2x Side IR)
        sensors.update();

        // Discover walls & re-flood distance field
        solver.updateWalls(
            currentX, currentY, currentHeading,
            sensors.hasWallFront(), sensors.hasWallLeft(), sensors.hasWallRight()
        );

        if (solver.isGoal(currentX, currentY)) break;

        Direction nextDir = solver.getNextMove(currentX, currentY, currentHeading);
        turnTo(nextDir);

        motors.moveForward(1);
        CellCoord nextCoord = FloodFill::advanceCoord(currentX, currentY, currentHeading);
        currentX = nextCoord.x;
        currentY = nextCoord.y;
        searchSteps++;
    }

    logMessage("Phase 1 Complete: Reached Center Goal at (" + 
               std::to_string(currentX) + "," + std::to_string(currentY) + 
               ") in " + std::to_string(searchSteps) + " steps.");
    API::setColor(currentX, currentY, 'G'); // Green for center

    // ========================================================================
    // PHASE 2: Return Run (Center Goal -> Start Cell (0,0))
    // Navigates back to origin, exploring unmapped corridors on the return.
    // ========================================================================
    logMessage("=== Phase 2: Return Run (Center Goal -> Start Cell (0,0)) ===");
    solver.setGoalsStart();
    int returnSteps = 0;

    while (currentX != START_X || currentY != START_Y) {
        if (checkReset()) break;

        solver.markVisited(currentX, currentY);
        if (!solver.isGoal(currentX, currentY)) {
            API::setColor(currentX, currentY, 'B'); // Blue for return run path
        }

        sensors.update();
        solver.updateWalls(
            currentX, currentY, currentHeading,
            sensors.hasWallFront(), sensors.hasWallLeft(), sensors.hasWallRight()
        );

        if (currentX == START_X && currentY == START_Y) break;

        Direction nextDir = solver.getNextMove(currentX, currentY, currentHeading);
        turnTo(nextDir);

        motors.moveForward(1);
        CellCoord nextCoord = FloodFill::advanceCoord(currentX, currentY, currentHeading);
        currentX = nextCoord.x;
        currentY = nextCoord.y;
        returnSteps++;
    }

    // Orient facing NORTH at start cell
    turnTo(NORTH);
    API::setColor(START_X, START_Y, 'G');
    logMessage("Phase 2 Complete: Returned to Start (0,0) in " + 
               std::to_string(returnSteps) + " steps. Maze thoroughly mapped!");

    // ========================================================================
    // PHASE 3: Speed Run (Start Cell -> Center Goal)
    // Pre-computed optimal sprint with multi-cell straight bursts (ALG-5 / OPT-6).
    // ========================================================================
    logMessage("=== Phase 3: Speed Run (Championship Optimal Sprint) ===");
    solver.setGoalsCenter();

    int pathSteps = solver.computeSpeedPath(START_X, START_Y, NORTH);
    const PathStep* speedPath = solver.getSpeedPath();

    logMessage("Optimal path planned: " + std::to_string(pathSteps) + " macro-movements.");

    // Highlight planned speed-run trajectory in Orange
    {
        int px = START_X, py = START_Y;
        for (int i = 0; i < pathSteps; ++i) {
            for (int s = 0; s < speedPath[i].straightCells; ++s) {
                px += DIR_DX[speedPath[i].dir];
                py += DIR_DY[speedPath[i].dir];
                API::setColor(px, py, 'O');
            }
        }
    }

    int totalSpeedCells = 0;
    for (int i = 0; i < pathSteps; ++i) {
        if (checkReset()) break;

        const PathStep& step = speedPath[i];
        turnTo(step.dir);

        // Execute multi-cell straight burst in a single moveForward call!
        motors.moveForward(step.straightCells);
        currentX += DIR_DX[step.dir] * step.straightCells;
        currentY += DIR_DY[step.dir] * step.straightCells;
        totalSpeedCells += step.straightCells;

        logMessage("Speed Burst: " + std::to_string(step.straightCells) + 
                   " cell(s) -> now at (" + std::to_string(currentX) + "," + 
                   std::to_string(currentY) + ")");
    }

    API::setColor(currentX, currentY, 'G');
    logMessage("🏁 CHAMPIONSHIP RUN COMPLETE! Goal reached in " + 
               std::to_string(totalSpeedCells) + " cells across " + 
               std::to_string(pathSteps) + " speed bursts!");

    return 0;
}

#else

// ============================================================================
// ESP32 Physical Hardware Firmware Entry Point (PlatformIO / Arduino Framework)
// ============================================================================

#include <Arduino.h>

static SensorArray sensors;
static MotorController motors;
static FloodFill solver;
static int currentX = START_X;
static int currentY = START_Y;
static Direction currentHeading = NORTH;
static uint32_t lastControlTimeMs = 0;

void setup() {
    Serial.begin(115200);
    Serial.println(F("[ESP32 Micromouse] Booting Championship Autonomous Firmware..."));

    sensors.init(DEFAULT_PINS);
    motors.init(DEFAULT_PINS);
    solver.init(DEFAULT_MAZE_WIDTH, DEFAULT_MAZE_HEIGHT);

    lastControlTimeMs = millis();
    Serial.println(F("[ESP32 Micromouse] Setup complete. Ready."));
}

void loop() {
    uint32_t now = millis();

    // High-frequency control loop (100 Hz / 10 ms interval)
    if (now - lastControlTimeMs >= (1000 / CONTROL_LOOP_HZ)) {
        float dt = (now - lastControlTimeMs) / 1000.0f;
        lastControlTimeMs = now;

        sensors.update();

        if (sensors.hasTactileCollision()) {
            Serial.println(F("[EMERGENCY] Tactile bumper collision! Stopping motors."));
            motors.stop();
            sensors.clearTactileFlags();
            return;
        }

        motors.updateControlLoop(sensors.getCenteringError(), dt);
    }
}

#endif
