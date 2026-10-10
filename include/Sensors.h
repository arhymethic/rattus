#pragma once

#include "Config.h"
#include <cstdint>

/**
 * @brief Processed Hybrid Sensor Readings (2x Front ToF + 2x Side Analog IR).
 */
struct SensorReadings {
    // Dual Front Time-of-Flight Sensors (mm)
    float frontLeftToFDistMm;
    float frontRightToFDistMm;
    float frontAvgDistMm;
    float frontSkewMm;        // frontLeft - frontRight (used to detect angular misalignment against front walls)

    // Dual Side Analog IR Sensors (mm)
    float leftSideDistMm;
    float rightSideDistMm;
    float centeringErrorMm;   // Lateral error relative to corridor centerline

    // Binary wall presence flags
    bool wallFront;
    bool wallLeft;
    bool wallRight;

    // Tactile bumpers
    bool leftTactileTriggered;
    bool rightTactileTriggered;
};

/**
 * @brief Optimized Hybrid Sensor Array Manager.
 */
class SensorArray {
public:
    SensorArray();
    ~SensorArray() = default;

    void init(const HardwarePins& pins = DEFAULT_PINS);
    void update();

    // Accessors
    bool hasWallFront() const { return readings_.wallFront; }
    bool hasWallLeft() const { return readings_.wallLeft; }
    bool hasWallRight() const { return readings_.wallRight; }

    float getFrontDistance() const { return readings_.frontAvgDistMm; }
    float getFrontLeftToF() const { return readings_.frontLeftToFDistMm; }
    float getFrontRightToF() const { return readings_.frontRightToFDistMm; }
    float getFrontSkew() const { return readings_.frontSkewMm; }

    float getLeftSideDistance() const { return readings_.leftSideDistMm; }
    float getRightSideDistance() const { return readings_.rightSideDistMm; }
    float getCenteringError() const { return readings_.centeringErrorMm; }

    const SensorReadings& getReadings() const { return readings_; }

    void clearTactileFlags();
    bool hasTactileCollision() const { 
        return readings_.leftTactileTriggered || readings_.rightTactileTriggered; 
    }

private:
    HardwarePins pins_;
    SensorReadings readings_;
    float leftFilteredDistMm_;
    float rightFilteredDistMm_;

    float rawAdcToDistanceMm(int adcVal);
    void computeDifferentialCentering();
};
