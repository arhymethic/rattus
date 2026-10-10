#include "Sensors.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#ifdef SIMULATION
#include "API.h"
#endif

SensorArray::SensorArray()
    : pins_(DEFAULT_PINS),
      leftFilteredDistMm_(NOMINAL_SIDE_WALL_DIST_MM),
      rightFilteredDistMm_(NOMINAL_SIDE_WALL_DIST_MM) {
    readings_ = {
        .frontLeftToFDistMm = 999.0f,
        .frontRightToFDistMm = 999.0f,
        .frontAvgDistMm = 999.0f,
        .frontSkewMm = 0.0f,
        .leftSideDistMm = NOMINAL_SIDE_WALL_DIST_MM,
        .rightSideDistMm = NOMINAL_SIDE_WALL_DIST_MM,
        .centeringErrorMm = 0.0f,
        .wallFront = false,
        .wallLeft = false,
        .wallRight = false,
        .leftTactileTriggered = false,
        .rightTactileTriggered = false
    };
}

void SensorArray::init(const HardwarePins& pins) {
    pins_ = pins;
#ifdef SIMULATION
    std::cerr << "[Sensors] Initialized 2x Front ToF + 2x Side IR (SIMULATION via MMS API)." << std::endl;
#else
    std::cerr << "[Sensors] Initialized 2x Front ToF + 2x Side IR for ESP32." << std::endl;
    // Hardware I2C initialization for dual VL53L1X with XSHUT address assignment
    // ADC configuration for analog side IRs
#endif
}

void SensorArray::update() {
#ifdef SIMULATION
    // MMS simulation mode
    readings_.wallFront = API::wallFront();
    readings_.wallLeft  = API::wallLeft();
    readings_.wallRight = API::wallRight();

    // Synthesize realistic distance metrics for the dual front ToFs & dual side IRs
    if (readings_.wallFront) {
        readings_.frontLeftToFDistMm = TOF_FRONT_BRAKING_DIST_MM;
        readings_.frontRightToFDistMm = TOF_FRONT_BRAKING_DIST_MM;
        readings_.frontAvgDistMm = TOF_FRONT_BRAKING_DIST_MM;
        readings_.frontSkewMm = 0.0f;
    } else {
        readings_.frontLeftToFDistMm = 999.0f;
        readings_.frontRightToFDistMm = 999.0f;
        readings_.frontAvgDistMm = 999.0f;
        readings_.frontSkewMm = 0.0f;
    }

    readings_.leftSideDistMm = readings_.wallLeft ? NOMINAL_SIDE_WALL_DIST_MM : 999.0f;
    readings_.rightSideDistMm = readings_.wallRight ? NOMINAL_SIDE_WALL_DIST_MM : 999.0f;
    readings_.centeringErrorMm = 0.0f;
#else
    // Hardware Mode: Read Dual Front VL53L1X ToF Sensors
    // float distL = tofLeft.read();
    // float distR = tofRight.read();
    // readings_.frontLeftToFDistMm = distL;
    // readings_.frontRightToFDistMm = distR;
    readings_.frontAvgDistMm = (readings_.frontLeftToFDistMm + readings_.frontRightToFDistMm) * 0.5f;
    readings_.frontSkewMm = readings_.frontLeftToFDistMm - readings_.frontRightToFDistMm;
    readings_.wallFront = (readings_.frontAvgDistMm < TOF_FRONT_WALL_THRESHOLD_MM);

    // Read Dual Side Analog IR Sensors with EMA Low-Pass Filter (OPT-9)
    int leftAdc = 0;
    int rightAdc = 0;
    // if (pins_.irLeftAdcPin >= 0) leftAdc = analogRead(pins_.irLeftAdcPin);
    // if (pins_.irRightAdcPin >= 0) rightAdc = analogRead(pins_.irRightAdcPin);
    float rawLeftDist = rawAdcToDistanceMm(leftAdc);
    float rawRightDist = rawAdcToDistanceMm(rightAdc);

    constexpr float EMA_ALPHA = 0.35f;
    leftFilteredDistMm_ = (EMA_ALPHA * rawLeftDist) + ((1.0f - EMA_ALPHA) * leftFilteredDistMm_);
    rightFilteredDistMm_ = (EMA_ALPHA * rawRightDist) + ((1.0f - EMA_ALPHA) * rightFilteredDistMm_);

    readings_.leftSideDistMm = leftFilteredDistMm_;
    readings_.rightSideDistMm = rightFilteredDistMm_;

    readings_.wallLeft = (readings_.leftSideDistMm < SIDE_WALL_DETECTION_THRESHOLD_MM);
    readings_.wallRight = (readings_.rightSideDistMm < SIDE_WALL_DETECTION_THRESHOLD_MM);

    computeDifferentialCentering();
#endif
}

void SensorArray::computeDifferentialCentering() {
    if (readings_.wallLeft && readings_.wallRight) {
        // Differential centering in two-wall corridor
        readings_.centeringErrorMm = readings_.rightSideDistMm - readings_.leftSideDistMm;
    } else if (readings_.wallLeft) {
        readings_.centeringErrorMm = 2.0f * (NOMINAL_SIDE_WALL_DIST_MM - readings_.leftSideDistMm);
    } else if (readings_.wallRight) {
        readings_.centeringErrorMm = 2.0f * (readings_.rightSideDistMm - NOMINAL_SIDE_WALL_DIST_MM);
    } else {
        readings_.centeringErrorMm = 0.0f;
    }
}

float SensorArray::rawAdcToDistanceMm(int adcVal) {
    if (adcVal <= 80) return 999.0f;
    return 15000.0f / (static_cast<float>(adcVal) + 50.0f);
}

void SensorArray::clearTactileFlags() {
    readings_.leftTactileTriggered = false;
    readings_.rightTactileTriggered = false;
}
