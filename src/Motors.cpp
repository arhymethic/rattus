#include "Motors.h"
#include <algorithm>
#include <cmath>
#include <iostream>

#ifdef SIMULATION
#include "API.h"
#endif

// ============================================================================
// PIDController Implementation
// ============================================================================

PIDController::PIDController() 
    : gains_{0, 0, 0, 0, 0, 0}, prevError_(0.0f), integral_(0.0f) {}

PIDController::PIDController(const PIDGains& gains)
    : gains_(gains), prevError_(0.0f), integral_(0.0f) {}

void PIDController::setGains(const PIDGains& gains) {
    gains_ = gains;
    reset();
}

float PIDController::compute(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measured;

    // Integral term with anti-windup clamping
    integral_ += error * dt;
    if (gains_.integralLimit > 0.0f) {
        integral_ = std::clamp(integral_, -gains_.integralLimit, gains_.integralLimit);
    }

    // Derivative term
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

MotorController::MotorController()
    : pins_(DEFAULT_PINS),
      steeringPid_(STEERING_PID_GAINS),
      leftVelocityPid_(VELOCITY_PID_GAINS),
      rightVelocityPid_(VELOCITY_PID_GAINS),
      leftEncoderTicks_(0),
      rightEncoderTicks_(0),
      targetLinearVelocityMmS_(0.0f),
      targetAngularVelocityRadS_(0.0f) {}

void MotorController::init(const HardwarePins& pins) {
    pins_ = pins;
#ifdef SIMULATION
    std::cerr << "[Motors] Initialized in SIMULATION mode via MMS API." << std::endl;
#else
    std::cerr << "[Motors] Initialized in HARDWARE mode for ESP32." << std::endl;
    // Hardware PWM timer setup and encoder interrupt attachments
    // Will be finalized upon user confirmation of pin configuration
#endif
    resetOdometry();
}

void MotorController::moveForward(int cells) {
#ifdef SIMULATION
    API::moveForward(cells);
#else
    // Hardware cell traversal:
    // Drive target distance = cells * CELL_SIZE_MM using encoder counts and wall-centering PID
    std::cerr << "[Motors] Driving forward " << cells << " cell(s)." << std::endl;
#endif
}

void MotorController::turnLeft() {
#ifdef SIMULATION
    API::turnLeft();
#else
    std::cerr << "[Motors] Turning Left 90 degrees." << std::endl;
#endif
}

void MotorController::turnRight() {
#ifdef SIMULATION
    API::turnRight();
#else
    std::cerr << "[Motors] Turning Right 90 degrees." << std::endl;
#endif
}

void MotorController::turnAround() {
#ifdef SIMULATION
    API::turnRight();
    API::turnRight();
#else
    std::cerr << "[Motors] Turning Around 180 degrees." << std::endl;
#endif
}

void MotorController::stop() {
    targetLinearVelocityMmS_ = 0.0f;
    targetAngularVelocityRadS_ = 0.0f;
    setMotorSpeeds(0.0f, 0.0f);
}

void MotorController::updateControlLoop(float centeringErrorMm, float dt) {
#ifndef SIMULATION
    // In physical mode:
    // 1. Compute steering correction from differential side IR error
    float steeringCorrection = steeringPid_.compute(0.0f, centeringErrorMm, dt);

    // 2. Compute individual wheel target velocities (differential drive kinematics)
    // v_left = v - (omega * W / 2)
    // v_right = v + (omega * W / 2)
    float leftTarget = targetLinearVelocityMmS_ - (steeringCorrection * WHEEL_TRACK_MM * 0.5f);
    float rightTarget = targetLinearVelocityMmS_ + (steeringCorrection * WHEEL_TRACK_MM * 0.5f);

    // 3. Close loop with encoder feedback
    // float leftMeasured = ...
    // float rightMeasured = ...
    // float leftPwm = leftVelocityPid_.compute(leftTarget, leftMeasured, dt);
    // float rightPwm = rightVelocityPid_.compute(rightTarget, rightMeasured, dt);
    // setMotorSpeeds(leftPwm, rightPwm);
#else
    (void)centeringErrorMm;
    (void)dt;
#endif
}

void MotorController::setMotorSpeeds(float leftPwm, float rightPwm) {
#ifndef SIMULATION
    // Write PWM duty cycle and direction pins to motor driver (e.g. DRV8833 / TB6612)
    (void)leftPwm;
    (void)rightPwm;
#else
    (void)leftPwm;
    (void)rightPwm;
#endif
}

float MotorController::getDistanceTraveledMm() const {
    float avgTicks = static_cast<float>(leftEncoderTicks_ + rightEncoderTicks_) * 0.5f;
    float wheelCircumference = M_PI * WHEEL_DIAMETER_MM;
    return (avgTicks / ENCODER_TICKS_PER_REV) * wheelCircumference;
}

void MotorController::resetOdometry() {
    leftEncoderTicks_ = 0;
    rightEncoderTicks_ = 0;
    steeringPid_.reset();
    leftVelocityPid_.reset();
    rightVelocityPid_.reset();
}
