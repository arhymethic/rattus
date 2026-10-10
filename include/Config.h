#pragma once

#include <cstdint>
#include <cstddef>

// ============================================================================
// Micromouse Competition & Physical Constraints
// ============================================================================

// Fixed 10x10 Competition Grid
constexpr int MAX_MAZE_DIM = 10;
constexpr int MAX_CELLS = MAX_MAZE_DIM * MAX_MAZE_DIM; // 100 cells
constexpr int MAX_GOAL_CELLS = 4;                      // 2x2 central destination: (4,4), (4,5), (5,4), (5,5)

constexpr int DEFAULT_MAZE_WIDTH = 10;
constexpr int DEFAULT_MAZE_HEIGHT = 10;

constexpr int START_X = 0;
constexpr int START_Y = 0;

// Competition Regulations & Scoring Parameters
constexpr int MAX_COMPETITION_ATTEMPTS = 3;           // Search + 2 Speed Runs within time allocation
constexpr float COMPETITION_TIME_LIMIT_SEC = 360.0f;   // 6.0 minutes standard competition window
constexpr float TURN_PENALTY_WEIGHT = 2.0f;           // Equivalent cell penalty per 90° turn in Dijkstra

// Physical Maze Dimensions (in millimeters)
constexpr float CELL_SIZE_MM = 198.0f;
constexpr float PATH_WIDTH_MM = 192.0f;
constexpr float WALL_HEIGHT_MM = 100.0f;
constexpr float WALL_THICKNESS_MM = 6.0f;

// Robot Physical Envelope
constexpr float ROBOT_MAX_LENGTH_MM = 190.0f;
constexpr float ROBOT_MAX_WIDTH_MM = 190.0f;
constexpr float ROBOT_MAX_HEIGHT_MM = 90.0f;

// Nominal Robot Chassis
constexpr float ROBOT_CHASSIS_LENGTH_MM = 105.0f;
constexpr float ROBOT_CHASSIS_WIDTH_MM = 82.0f;
constexpr float ROBOT_CHASSIS_HEIGHT_MM = 42.0f;
constexpr float WHEEL_DIAMETER_MM = 32.0f;
constexpr float WHEEL_TRACK_MM = 74.0f;
constexpr float ENCODER_TICKS_PER_REV = 360.0f;

// ============================================================================
// Direction Definitions & Bitmask Operations
// ============================================================================
enum Direction : uint8_t {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    DIR_COUNT = 4
};

// Wall Bitmasks (bits 0-3 only)
constexpr uint8_t WALL_NORTH = 1 << 0;
constexpr uint8_t WALL_EAST  = 1 << 1;
constexpr uint8_t WALL_SOUTH = 1 << 2;
constexpr uint8_t WALL_WEST  = 1 << 3;

// Direction deltas and masks
constexpr int DIR_DX[4] = { 0,  1,  0, -1};
constexpr int DIR_DY[4] = { 1,  0, -1,  0};
constexpr uint8_t DIR_WALL_MASK[4] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};

// Relative Turn Commands
enum TurnCommand : uint8_t {
    TURN_NONE = 0,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_AROUND
};

// Navigation Run Phases (3-phase championship structure)
enum class NavPhase : uint8_t {
    SEARCH_TO_CENTER,   // Phase 1: Explore from start to center goal
    RETURN_TO_START,    // Phase 2: Return to (0,0), discovering more walls
    SPEED_RUN,          // Phase 3: Sprint cached optimal path at max speed
    FINISHED
};

// ============================================================================
// Sensor Configuration: 2x Front Time-of-Flight + 2x Side Analog IR
// ============================================================================
constexpr float NOMINAL_SIDE_WALL_DIST_MM = 55.0f;
constexpr float SIDE_WALL_DETECTION_THRESHOLD_MM = 95.0f;

constexpr float TOF_FRONT_WALL_THRESHOLD_MM = 160.0f;
constexpr float TOF_FRONT_BRAKING_DIST_MM = 45.0f;
constexpr float TOF_FRONT_SKEW_MAX_MM = 15.0f;

// ============================================================================
// Control Loop & PID Parameters
// ============================================================================
struct PIDGains {
    float kp;
    float ki;
    float kd;
    float maxOutput;
    float minOutput;
    float integralLimit;
};

constexpr PIDGains STEERING_PID_GAINS = {
    .kp = 2.0f, .ki = 0.02f, .kd = 0.50f,
    .maxOutput = 120.0f, .minOutput = -120.0f, .integralLimit = 30.0f
};

constexpr PIDGains VELOCITY_PID_GAINS = {
    .kp = 2.8f, .ki = 0.10f, .kd = 0.15f,
    .maxOutput = 255.0f, .minOutput = -255.0f, .integralLimit = 50.0f
};

constexpr uint32_t CONTROL_LOOP_HZ = 100;
constexpr float CONTROL_LOOP_DT_SEC = 1.0f / CONTROL_LOOP_HZ;

// ============================================================================
// Hardware Pin Mappings (PLACEHOLDER — awaiting user schematic)
// ============================================================================
struct HardwarePins {
    int motorLeftPwm;
    int motorLeftDir;
    int motorRightPwm;
    int motorRightDir;

    int encoderLeftA;
    int encoderLeftB;
    int encoderRightA;
    int encoderRightB;

    int irLeftAdcPin;
    int irRightAdcPin;
    int irEmitterPin;

    int i2cSdaPin;
    int i2cSclPin;
    int tofFrontLeftXshutPin;
    int tofFrontRightXshutPin;

    int bumperLeftPin;
    int bumperRightPin;
    int ledStatusPin;
    int buttonStartPin;
};

constexpr HardwarePins DEFAULT_PINS = {
    .motorLeftPwm = -1, .motorLeftDir = -1,
    .motorRightPwm = -1, .motorRightDir = -1,
    .encoderLeftA = -1, .encoderLeftB = -1,
    .encoderRightA = -1, .encoderRightB = -1,
    .irLeftAdcPin = -1, .irRightAdcPin = -1, .irEmitterPin = -1,
    .i2cSdaPin = -1, .i2cSclPin = -1,
    .tofFrontLeftXshutPin = -1, .tofFrontRightXshutPin = -1,
    .bumperLeftPin = -1, .bumperRightPin = -1,
    .ledStatusPin = -1, .buttonStartPin = -1
};
