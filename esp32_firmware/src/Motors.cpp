#include "Motors.h"
#include <cmath>
#include <algorithm>

// ============================================================================
// PIDController Implementation
// ============================================================================

PIDController::PIDController()
    : gains_({0, 0, 0, 255, -255, 50}), prevError_(0.0f), integral_(0.0f) {}

PIDController::PIDController(const PIDGains& gains)
    : gains_(gains), prevError_(0.0f), integral_(0.0f) {}

void PIDController::setGains(const PIDGains& gains) {
    gains_ = gains;
    reset();
}

float PIDController::compute(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;
    float error = setpoint - measured;

    integral_ += error * dt;
    if (gains_.integralLimit > 0.0f) {
        integral_ = std::clamp(integral_, -gains_.integralLimit, gains_.integralLimit);
    }

    float derivative = (error - prevError_) / dt;
    prevError_ = error;

    float output = (gains_.kp * error) + (gains_.ki * integral_) + (gains_.kd * derivative);
    return std::clamp(output, gains_.minOutput, gains_.maxOutput);
}

void PIDController::reset() {
    prevError_ = 0.0f;
    integral_ = 0.0f;
}

// ============================================================================
// MotorController Implementation
// ============================================================================

MotorController* g_motorControllerInstance = nullptr;

constexpr int LEDC_CHANNEL_LEFT  = 0;
constexpr int LEDC_CHANNEL_RIGHT = 1;
constexpr int LEDC_FREQ_HZ       = 20000; // 20 kHz ultrasonic PWM (no audible whine)
constexpr int LEDC_RESOLUTION    = 8;     // 8-bit resolution (0-255 PWM duty)

MotorController::MotorController()
    : leftEncoderTicks_(0),
      rightEncoderTicks_(0),
      steeringPid_(STEERING_PID_GAINS),
      leftSpeedPid_(VELOCITY_PID_GAINS),
      rightSpeedPid_(VELOCITY_PID_GAINS),
      currentBasePwm_(0) {
    g_motorControllerInstance = this;
}

void IRAM_ATTR MotorController::isrLeftEncoderA() {
    if (g_motorControllerInstance) {
        int bVal = digitalRead(PIN_ENCODER_LEFT_B);
        if (bVal == HIGH) {
            g_motorControllerInstance->leftEncoderTicks_++;
        } else {
            g_motorControllerInstance->leftEncoderTicks_--;
        }
    }
}

void IRAM_ATTR MotorController::isrRightEncoderA() {
    if (g_motorControllerInstance) {
        int bVal = digitalRead(PIN_ENCODER_RIGHT_B);
        if (bVal == LOW) { // Opposite polarity for right wheel
            g_motorControllerInstance->rightEncoderTicks_++;
        } else {
            g_motorControllerInstance->rightEncoderTicks_--;
        }
    }
}

void MotorController::init() {
    Serial.println(F("[Motors] Configuring ESP32 LEDC PWM and Quadrature Encoder ISRs..."));

    // 1. Motor Direction Pins
    if (PIN_MOTOR_LEFT_DIR >= 0)  pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
    if (PIN_MOTOR_RIGHT_DIR >= 0) pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);

    // 2. Hardware LEDC PWM Timers (ESP32 Arduino core)
    ledcSetup(LEDC_CHANNEL_LEFT, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_RIGHT, LEDC_FREQ_HZ, LEDC_RESOLUTION);

    if (PIN_MOTOR_LEFT_PWM >= 0)  ledcAttachPin(PIN_MOTOR_LEFT_PWM, LEDC_CHANNEL_LEFT);
    if (PIN_MOTOR_RIGHT_PWM >= 0) ledcAttachPin(PIN_MOTOR_RIGHT_PWM, LEDC_CHANNEL_RIGHT);

    // 3. Encoder Interrupts
    if (PIN_ENCODER_LEFT_A >= 0 && PIN_ENCODER_LEFT_B >= 0) {
        pinMode(PIN_ENCODER_LEFT_A, INPUT_PULLUP);
        pinMode(PIN_ENCODER_LEFT_B, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_A), isrLeftEncoderA, RISING);
    }
    if (PIN_ENCODER_RIGHT_A >= 0 && PIN_ENCODER_RIGHT_B >= 0) {
        pinMode(PIN_ENCODER_RIGHT_A, INPUT_PULLUP);
        pinMode(PIN_ENCODER_RIGHT_B, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_A), isrRightEncoderA, RISING);
    }

    stop();
    resetOdometry();
    Serial.println(F("[Motors] Motor Controller Initialized."));
}

void MotorController::setMotorSpeeds(int leftPwm, int rightPwm) {
    leftPwm = std::clamp(leftPwm, -255, 255);
    rightPwm = std::clamp(rightPwm, -255, 255);

    // Left Motor
    if (leftPwm >= 0) {
        if (PIN_MOTOR_LEFT_DIR >= 0) digitalWrite(PIN_MOTOR_LEFT_DIR, HIGH);
        ledcWrite(LEDC_CHANNEL_LEFT, leftPwm);
    } else {
        if (PIN_MOTOR_LEFT_DIR >= 0) digitalWrite(PIN_MOTOR_LEFT_DIR, LOW);
        ledcWrite(LEDC_CHANNEL_LEFT, -leftPwm);
    }

    // Right Motor
    if (rightPwm >= 0) {
        if (PIN_MOTOR_RIGHT_DIR >= 0) digitalWrite(PIN_MOTOR_RIGHT_DIR, HIGH);
        ledcWrite(LEDC_CHANNEL_RIGHT, rightPwm);
    } else {
        if (PIN_MOTOR_RIGHT_DIR >= 0) digitalWrite(PIN_MOTOR_RIGHT_DIR, LOW);
        ledcWrite(LEDC_CHANNEL_RIGHT, -rightPwm);
    }
}

void MotorController::stop() {
    currentBasePwm_ = 0;
    setMotorSpeeds(0, 0);
}

void MotorController::resetOdometry() {
    noInterrupts();
    leftEncoderTicks_ = 0;
    rightEncoderTicks_ = 0;
    interrupts();
    steeringPid_.reset();
}

float MotorController::getDistanceTraveledMm() const {
    int32_t avgTicks = (leftEncoderTicks_ + rightEncoderTicks_) / 2;
    return static_cast<float>(avgTicks) * MM_PER_ENCODER_TICK;
}

void MotorController::updateCenteringLoop(float centeringErrorMm, float dt) {
    if (currentBasePwm_ == 0) return;

    // Differential steering correction: positive steering error turns right
    float steerCorrection = steeringPid_.compute(0.0f, centeringErrorMm, dt);

    int leftPwm  = static_cast<int>(currentBasePwm_ - steerCorrection);
    int rightPwm = static_cast<int>(currentBasePwm_ + steerCorrection);

    setMotorSpeeds(leftPwm, rightPwm);
}

// ----------------------------------------------------------------------------
// Discrete Motion Primitives
// ----------------------------------------------------------------------------

void MotorController::moveForward(int cells, int basePwm) {
    currentBasePwm_ = basePwm;
    resetOdometry();

    float targetDistMm = cells * CELL_SIZE_MM;
    float targetTicks  = targetDistMm / MM_PER_ENCODER_TICK;

    uint32_t lastLoop = millis();

    while (true) {
        int32_t currentTicks = (leftEncoderTicks_ + rightEncoderTicks_) / 2;
        if (currentTicks >= targetTicks) {
            break;
        }

        uint32_t now = millis();
        if (now - lastLoop >= 10) { // 100 Hz step
            float dt = (now - lastLoop) / 1000.0f;
            lastLoop = now;

            // Deceleration ramp in the last 40 mm to prevent overshooting
            float remainingMm = (targetTicks - currentTicks) * MM_PER_ENCODER_TICK;
            int effectivePwm = currentBasePwm_;
            if (remainingMm < 40.0f && effectivePwm > 90) {
                effectivePwm = 90 + static_cast<int>((remainingMm / 40.0f) * (currentBasePwm_ - 90));
            }

            setMotorSpeeds(effectivePwm, effectivePwm);
        }
        delay(1);
    }

    stop();
    delay(40); // Settle
}

void MotorController::turnLeft() {
    stop();
    resetOdometry();

    // Turn 90 degrees in-place: arc length = (pi * trackWidth) / 4
    float arcMm = (3.14159265f * WHEEL_TRACK_MM) / 4.0f;
    int32_t targetTicks = static_cast<int32_t>(arcMm / MM_PER_ENCODER_TICK);

    setMotorSpeeds(-SPEED_TURN_PWM, SPEED_TURN_PWM);

    while (std::abs(rightEncoderTicks_) < targetTicks) {
        delay(1);
    }

    stop();
    delay(40);
}

void MotorController::turnRight() {
    stop();
    resetOdometry();

    float arcMm = (3.14159265f * WHEEL_TRACK_MM) / 4.0f;
    int32_t targetTicks = static_cast<int32_t>(arcMm / MM_PER_ENCODER_TICK);

    setMotorSpeeds(SPEED_TURN_PWM, -SPEED_TURN_PWM);

    while (std::abs(leftEncoderTicks_) < targetTicks) {
        delay(1);
    }

    stop();
    delay(40);
}

void MotorController::turnAround() {
    stop();
    resetOdometry();

    float arcMm = (3.14159265f * WHEEL_TRACK_MM) / 2.0f; // 180 degrees
    int32_t targetTicks = static_cast<int32_t>(arcMm / MM_PER_ENCODER_TICK);

    setMotorSpeeds(SPEED_TURN_PWM, -SPEED_TURN_PWM);

    while (std::abs(leftEncoderTicks_) < targetTicks) {
        delay(1);
    }

    stop();
    delay(40);
}
