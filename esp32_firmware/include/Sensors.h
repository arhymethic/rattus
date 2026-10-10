#pragma once

#include "Config.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

/**
 * @brief Processed Hybrid Sensor Readings.
 */
struct SensorReadings {
    // Dual Front VL53L1X ToF Sensors (in mm)
    float frontLeftToFDistMm;
    float frontRightToFDistMm;
    float frontAvgDistMm;
    float frontSkewMm;        // frontLeft - frontRight (skew against front wall)

    // Dual Side Analog IR Sensors (in mm)
    float leftSideDistMm;
    float rightSideDistMm;
    float centeringErrorMm;   // Lateral error relative to corridor centerline

    // Binary wall presence flags
    bool wallFront;
    bool wallLeft;
    bool wallRight;

    // Tactile bumper flags
    volatile bool leftTactileTriggered;
    volatile bool rightTactileTriggered;
};

/**
 * @brief Hardware Sensor Array Manager for Physical ESP32 Micromouse.
 */
class SensorArray {
public:
    SensorArray();
    ~SensorArray() = default;

    /**
     * @brief Initialize I2C bus, dual VL53L1X addresses via XSHUT, ADC, and bumper ISRs.
     */
    bool init();

    /**
     * @brief Poll all physical sensors (called in 100 Hz control loop or step loop).
     */
    void update();

    // Wall presence flags
    bool hasWallFront() const { return readings_.wallFront; }
    bool hasWallLeft()  const { return readings_.wallLeft; }
    bool hasWallRight() const { return readings_.wallRight; }

    // Distance accessors
    float getFrontDistance()     const { return readings_.frontAvgDistMm; }
    float getFrontLeftToF()      const { return readings_.frontLeftToFDistMm; }
    float getFrontRightToF()     const { return readings_.frontRightToFDistMm; }
    float getFrontSkew()         const { return readings_.frontSkewMm; }

    float getLeftSideDistance()  const { return readings_.leftSideDistMm; }
    float getRightSideDistance() const { return readings_.rightSideDistMm; }
    float getCenteringError()    const { return readings_.centeringErrorMm; }

    const SensorReadings& getReadings() const { return readings_; }

    void clearTactileFlags();
    bool hasTactileCollision() const {
        return readings_.leftTactileTriggered || readings_.rightTactileTriggered;
    }

    // Static ISR handlers for tactile bumpers
    static void IRAM_ATTR isrLeftBumper();
    static void IRAM_ATTR isrRightBumper();

private:
    VL53L1X tofLeft_;
    VL53L1X tofRight_;

    SensorReadings readings_;
    float leftFilteredDistMm_;
    float rightFilteredDistMm_;

    float rawAdcToDistanceMm(int adcVal);
    void computeDifferentialCentering();
};

extern SensorArray* g_sensorArrayInstance;
