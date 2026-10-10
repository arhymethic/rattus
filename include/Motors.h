#pragma once

#include "Config.h"
#include <cstdint>

/**
 * @brief General Discrete PID Controller with anti-windup.
 */
class PIDController {
public:
    PIDController();
    explicit PIDController(const PIDGains& gains);

    void setGains(const PIDGains& gains);
    float compute(float setpoint, float measured, float dt);
    void reset();

private:
    PIDGains gains_;
    float prevError_;
    float integral_;
};

/**
 * @brief Differential Drive DC Motor Controller with Encoder Odometry and MMS Bridge.
 */
class MotorController {
public:
    MotorController();
    ~MotorController() = default;

    /**
     * @brief Initialize motor PWM timers, direction pins, and encoder ISRs.
     * @param pins Hardware pin assignments (ignored in simulation mode).
     */
    void init(const HardwarePins& pins = DEFAULT_PINS);

    /**
     * @brief High-level discrete maze motions.
     */
    void moveForward(int cells = 1);
    void turnLeft();
    void turnRight();
    void turnAround();
    void stop();

    /**
     * @brief Update low-level control loop step (100 Hz).
     * Computes PID corrections, injects differential IR steering, and sets motor PWMs.
     */
    void updateControlLoop(float centeringErrorMm, float dt = CONTROL_LOOP_DT_SEC);

    // Odometry & state accessors
    int32_t getLeftEncoderTicks() const { return leftEncoderTicks_; }
    int32_t getRightEncoderTicks() const { return rightEncoderTicks_; }
    float getDistanceTraveledMm() const;
    void resetOdometry();

private:
    HardwarePins pins_;
    PIDController steeringPid_;
    PIDController leftVelocityPid_;
    PIDController rightVelocityPid_;

    volatile int32_t leftEncoderTicks_;
    volatile int32_t rightEncoderTicks_;

    float targetLinearVelocityMmS_;
    float targetAngularVelocityRadS_;

    void setMotorSpeeds(float leftPwm, float rightPwm);
};
