#pragma once

#include "Config.h"
#include <Arduino.h>

/**
 * @brief Discrete PID Controller with integral anti-windup clamping.
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
 * @brief Differential Drive Motor Controller with Hardware LEDC PWM and Quadrature Encoders.
 */
class MotorController {
public:
    MotorController();
    ~MotorController() = default;

    /**
     * @brief Initialize ESP32 LEDC PWM timers, direction GPIOs, and encoder ISRs.
     */
    void init();

    /**
     * @brief High-level discrete maze motions governed by closed-loop encoder feedback.
     */
    void moveForward(int cells = 1, int basePwm = SPEED_SEARCH_PWM);
    void turnLeft();
    void turnRight();
    void turnAround();
    void stop();

    /**
     * @brief Set direct raw motor PWM and direction (-255 to 255).
     */
    void setMotorSpeeds(int leftPwm, int rightPwm);

    /**
     * @brief Update differential wall-centering control loop (called in 100 Hz timer).
     */
    void updateCenteringLoop(float centeringErrorMm, float dt = CONTROL_LOOP_DT_SEC);

    // Odometry & encoder accessors
    int32_t getLeftTicks() const { return leftEncoderTicks_; }
    int32_t getRightTicks() const { return rightEncoderTicks_; }
    float getDistanceTraveledMm() const;
    void resetOdometry();

    // Static ISRs for quadrature encoders
    static void IRAM_ATTR isrLeftEncoderA();
    static void IRAM_ATTR isrRightEncoderA();

private:
    volatile int32_t leftEncoderTicks_;
    volatile int32_t rightEncoderTicks_;

    PIDController steeringPid_;
    PIDController leftSpeedPid_;
    PIDController rightSpeedPid_;

    int currentBasePwm_;
};

extern MotorController* g_motorControllerInstance;
