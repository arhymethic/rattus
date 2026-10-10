#pragma once

#include <cstdint>
#include <cstddef>

// ============================================================================
// 1. ESP32 HARDWARE PIN ASSIGNMENTS (Customizable for your schematic)
// ============================================================================

// --- DC Motors (H-Bridge: TB6612FNG or DRV8833) ---
constexpr int PIN_MOTOR_LEFT_PWM      = 25;  // ESP32 LEDC PWM channel 0
constexpr int PIN_MOTOR_LEFT_DIR      = 26;  // Motor Left Direction GPIO
constexpr int PIN_MOTOR_RIGHT_PWM     = 27;  // ESP32 LEDC PWM channel 1
constexpr int PIN_MOTOR_RIGHT_DIR     = 14;  // Motor Right Direction GPIO

// --- Quadrature Encoders (Optical or Magnetic Hall Encoders) ---
constexpr int PIN_ENCODER_LEFT_A      = 34;  // Left Encoder Phase A (Input only)
constexpr int PIN_ENCODER_LEFT_B      = 35;  // Left Encoder Phase B (Input only)
constexpr int PIN_ENCODER_RIGHT_A     = 36;  // Right Encoder Phase A (Input only - SENSOR_VP)
constexpr int PIN_ENCODER_RIGHT_B     = 39;  // Right Encoder Phase B (Input only - SENSOR_VN)

// --- Dual Side Analog IR Distance Sensors (Sharp GP2Y0A51SK0F or Discrete IR) ---
constexpr int PIN_IR_LEFT_ADC         = 32;  // Left Side IR (ADC1_CH4, safe with Wi-Fi/BT)
constexpr int PIN_IR_RIGHT_ADC        = 33;  // Right Side IR (ADC1_CH5, safe with Wi-Fi/BT)
constexpr int PIN_IR_EMITTER_ENABLE   = 13;  // Optional: Emitter modulation MOSFET pin (-1 if tied to VCC)

// --- Dual Front Time-of-Flight Sensors (VL53L1X via I2C) ---
constexpr int PIN_I2C_SDA             = 21;  // ESP32 default I2C Data
constexpr int PIN_I2C_SCL             = 22;  // ESP32 default I2C Clock
constexpr int PIN_TOF_LEFT_XSHUT      = 18;  // Left VL53L1X Shutdown / Address change pin
constexpr int PIN_TOF_RIGHT_XSHUT     = 19;  // Right VL53L1X Shutdown / Address change pin

// --- Tactile Bumper Switches (Emergency Collision Protection) ---
constexpr int PIN_BUMPER_LEFT         = 4;   // Left bumper microswitch (Active LOW with internal pull-up)
constexpr int PIN_BUMPER_RIGHT        = 5;   // Right bumper microswitch (Active LOW with internal pull-up)

// --- User Interface & Diagnostics ---
constexpr int PIN_START_BUTTON        = 23;  // Physical push-button to start run (Active LOW)
constexpr int PIN_STATUS_LED          = 2;   // Onboard diagnostic LED (Blinks on search, solid on speed run)
constexpr int PIN_BUZZER              = 12;  // Piezo buzzer for audio telemetry (-1 if omitted)

// ============================================================================
// 2. 10x10 Competition Maze Specifications
// ============================================================================
constexpr int MAX_MAZE_DIM            = 10;
constexpr int MAX_CELLS               = 100; // 10 x 10 grid
constexpr int MAX_GOAL_CELLS          = 4;   // 2x2 Center Goals: (4,4), (4,5), (5,4), (5,5)

constexpr int DEFAULT_MAZE_WIDTH      = 10;
constexpr int DEFAULT_MAZE_HEIGHT     = 10;

constexpr int START_X                 = 0;
constexpr int START_Y                 = 0;

// Physical Dimensions (in millimeters)
constexpr float CELL_SIZE_MM          = 198.0f; // Center-to-center cell pitch
constexpr float PATH_WIDTH_MM         = 192.0f; // Corridor width between walls
constexpr float WALL_HEIGHT_MM        = 100.0f; // Barrier height (6 mm thickness)
constexpr float WALL_THICKNESS_MM     = 6.0f;

// Robot Physical Envelope Limits (Must comply with regulations)
constexpr float ROBOT_MAX_LENGTH_MM   = 190.0f; // 19 cm limit
constexpr float ROBOT_MAX_WIDTH_MM    = 190.0f; // 19 cm limit
constexpr float ROBOT_MAX_HEIGHT_MM   = 90.0f;  // < 90 mm limit

// Robot Mechanical Geometry
constexpr float WHEEL_DIAMETER_MM     = 32.0f;  // Rubber/silicone wheel diameter
constexpr float WHEEL_CIRCUMFERENCE_MM = 3.14159265f * WHEEL_DIAMETER_MM;
constexpr float WHEEL_TRACK_MM        = 74.0f;  // Distance between left and right contact patches
constexpr float ENCODER_TICKS_PER_REV = 360.0f; // Total quadrature encoder counts per wheel turn
constexpr float MM_PER_ENCODER_TICK   = WHEEL_CIRCUMFERENCE_MM / ENCODER_TICKS_PER_REV;

// ============================================================================
// 3. Direction Definitions & Bitmask Operations
// ============================================================================
enum Direction : uint8_t {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3,
    DIR_COUNT = 4
};

constexpr uint8_t WALL_NORTH = 1 << 0;
constexpr uint8_t WALL_EAST  = 1 << 1;
constexpr uint8_t WALL_SOUTH = 1 << 2;
constexpr uint8_t WALL_WEST  = 1 << 3;

constexpr int DIR_DX[4] = { 0,  1,  0, -1 };
constexpr int DIR_DY[4] = { 1,  0, -1,  0 };
constexpr uint8_t DIR_WALL_MASK[4] = { WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST };

enum TurnCommand : uint8_t {
    TURN_NONE = 0,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_AROUND
};

enum class NavPhase : uint8_t {
    SEARCH_TO_CENTER,
    RETURN_TO_START,
    SPEED_RUN,
    FINISHED
};

// ============================================================================
// 4. Sensor Calibration & Thresholds
// ============================================================================
// Side Analog IRs (Corridor Centering)
constexpr float NOMINAL_SIDE_WALL_DIST_MM        = 55.0f; // Ideal distance when centered in 192 mm path
constexpr float SIDE_WALL_DETECTION_THRESHOLD_MM = 95.0f; // Wall present if reading < 95 mm
constexpr float SENSOR_EMA_ALPHA                 = 0.35f; // Low-pass filter smoothing coefficient

// Front ToFs (Longitudinal Braking & Skew Alignment)
constexpr float TOF_FRONT_WALL_THRESHOLD_MM      = 160.0f; // Detect wall in upcoming cell
constexpr float TOF_FRONT_BRAKING_DIST_MM        = 45.0f;  // Center-cell stopping alignment distance
constexpr float TOF_FRONT_SKEW_MAX_MM            = 12.0f;  // Angular skew correction threshold

// ============================================================================
// 5. Control Loop & PID Gains
// ============================================================================
struct PIDGains {
    float kp;
    float ki;
    float kd;
    float maxOutput;
    float minOutput;
    float integralLimit;
};

// Differential Steering PID (keeps bot centered between walls)
constexpr PIDGains STEERING_PID_GAINS = {
    .kp = 2.2f, .ki = 0.02f, .kd = 0.60f,
    .maxOutput = 100.0f, .minOutput = -100.0f, .integralLimit = 25.0f
};

// Motor Velocity PID (governs straight driving and turns)
constexpr PIDGains VELOCITY_PID_GAINS = {
    .kp = 3.0f, .ki = 0.12f, .kd = 0.18f,
    .maxOutput = 255.0f, .minOutput = -255.0f, .integralLimit = 50.0f
};

constexpr uint32_t CONTROL_LOOP_HZ     = 100;
constexpr float CONTROL_LOOP_DT_SEC    = 1.0f / CONTROL_LOOP_HZ;

// Speed Settings (PWM values 0 - 255)
constexpr int SPEED_SEARCH_PWM         = 130;  // Safe exploration speed (~0.4 m/s)
constexpr int SPEED_SPRINT_PWM         = 240;  // Maximum speed run sprint (~1.2 m/s)
constexpr int SPEED_TURN_PWM           = 110;  // In-place turn speed
