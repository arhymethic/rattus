#include "Sensors.h"

SensorArray* g_sensorArrayInstance = nullptr;

SensorArray::SensorArray()
    : leftFilteredDistMm_(NOMINAL_SIDE_WALL_DIST_MM),
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
    g_sensorArrayInstance = this;
}

void IRAM_ATTR SensorArray::isrLeftBumper() {
    if (g_sensorArrayInstance) {
        g_sensorArrayInstance->readings_.leftTactileTriggered = true;
    }
}

void IRAM_ATTR SensorArray::isrRightBumper() {
    if (g_sensorArrayInstance) {
        g_sensorArrayInstance->readings_.rightTactileTriggered = true;
    }
}

bool SensorArray::init() {
    Serial.println(F("[Sensors] Initializing ESP32 Hybrid Sensor Array..."));

    // 1. Configure Tactile Bumper Pins & Interrupts
    if (PIN_BUMPER_LEFT >= 0) {
        pinMode(PIN_BUMPER_LEFT, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PIN_BUMPER_LEFT), isrLeftBumper, FALLING);
    }
    if (PIN_BUMPER_RIGHT >= 0) {
        pinMode(PIN_BUMPER_RIGHT, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PIN_BUMPER_RIGHT), isrRightBumper, FALLING);
    }

    // 2. Configure Analog IR Pins
    if (PIN_IR_LEFT_ADC >= 0) pinMode(PIN_IR_LEFT_ADC, INPUT);
    if (PIN_IR_RIGHT_ADC >= 0) pinMode(PIN_IR_RIGHT_ADC, INPUT);
    if (PIN_IR_EMITTER_ENABLE >= 0) {
        pinMode(PIN_IR_EMITTER_ENABLE, OUTPUT);
        digitalWrite(PIN_IR_EMITTER_ENABLE, HIGH); // Enable IR LEDs
    }

    // 3. Configure Dual VL53L1X Time-of-Flight Sensors via I2C & XSHUT
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000); // 400 kHz Fast I2C

    if (PIN_TOF_LEFT_XSHUT >= 0) {
        pinMode(PIN_TOF_LEFT_XSHUT, OUTPUT);
        digitalWrite(PIN_TOF_LEFT_XSHUT, LOW); // Hold Left in reset
    }
    if (PIN_TOF_RIGHT_XSHUT >= 0) {
        pinMode(PIN_TOF_RIGHT_XSHUT, OUTPUT);
        digitalWrite(PIN_TOF_RIGHT_XSHUT, LOW); // Hold Right in reset
    }
    delay(10);

    // Boot Left VL53L1X and assign address 0x30
    if (PIN_TOF_LEFT_XSHUT >= 0) {
        digitalWrite(PIN_TOF_LEFT_XSHUT, HIGH);
        delay(15);
        tofLeft_.setTimeout(500);
        if (tofLeft_.init()) {
            tofLeft_.setAddress(0x30);
            tofLeft_.setDistanceMode(VL53L1X::Short);
            tofLeft_.setMeasurementTimingBudget(20000); // 20 ms fast budget
            tofLeft_.startContinuous(25);
            Serial.println(F("[Sensors] Front Left VL53L1X initialized at I2C 0x30."));
        } else {
            Serial.println(F("[Sensors] WARNING: Front Left VL53L1X failed to initialize!"));
        }
    }

    // Boot Right VL53L1X and assign address 0x31
    if (PIN_TOF_RIGHT_XSHUT >= 0) {
        digitalWrite(PIN_TOF_RIGHT_XSHUT, HIGH);
        delay(15);
        tofRight_.setTimeout(500);
        if (tofRight_.init()) {
            tofRight_.setAddress(0x31);
            tofRight_.setDistanceMode(VL53L1X::Short);
            tofRight_.setMeasurementTimingBudget(20000); // 20 ms fast budget
            tofRight_.startContinuous(25);
            Serial.println(F("[Sensors] Front Right VL53L1X initialized at I2C 0x31."));
        } else {
            Serial.println(F("[Sensors] WARNING: Front Right VL53L1X failed to initialize!"));
        }
    }

    Serial.println(F("[Sensors] Sensor Array Initialization Complete."));
    return true;
}

void SensorArray::update() {
    // 1. Read Dual Front VL53L1X Time-of-Flight Sensors
    uint16_t dL = tofLeft_.read(false);
    uint16_t dR = tofRight_.read(false);

    if (!tofLeft_.timeoutOccurred() && dL > 0 && dL < 2000) {
        readings_.frontLeftToFDistMm = static_cast<float>(dL);
    } else {
        readings_.frontLeftToFDistMm = 999.0f;
    }

    if (!tofRight_.timeoutOccurred() && dR > 0 && dR < 2000) {
        readings_.frontRightToFDistMm = static_cast<float>(dR);
    } else {
        readings_.frontRightToFDistMm = 999.0f;
    }

    readings_.frontAvgDistMm = (readings_.frontLeftToFDistMm + readings_.frontRightToFDistMm) * 0.5f;
    readings_.frontSkewMm    = readings_.frontLeftToFDistMm - readings_.frontRightToFDistMm;
    readings_.wallFront      = (readings_.frontAvgDistMm < TOF_FRONT_WALL_THRESHOLD_MM);

    // 2. Read Dual Side Analog IR Distance Sensors with Low-Pass EMA Filter
    int rawLeftAdc = (PIN_IR_LEFT_ADC >= 0) ? analogRead(PIN_IR_LEFT_ADC) : 0;
    int rawRightAdc = (PIN_IR_RIGHT_ADC >= 0) ? analogRead(PIN_IR_RIGHT_ADC) : 0;

    float rawLeftDist = rawAdcToDistanceMm(rawLeftAdc);
    float rawRightDist = rawAdcToDistanceMm(rawRightAdc);

    leftFilteredDistMm_  = (SENSOR_EMA_ALPHA * rawLeftDist) + ((1.0f - SENSOR_EMA_ALPHA) * leftFilteredDistMm_);
    rightFilteredDistMm_ = (SENSOR_EMA_ALPHA * rawRightDist) + ((1.0f - SENSOR_EMA_ALPHA) * rightFilteredDistMm_);

    readings_.leftSideDistMm  = leftFilteredDistMm_;
    readings_.rightSideDistMm = rightFilteredDistMm_;

    readings_.wallLeft  = (readings_.leftSideDistMm < SIDE_WALL_DETECTION_THRESHOLD_MM);
    readings_.wallRight = (readings_.rightSideDistMm < SIDE_WALL_DETECTION_THRESHOLD_MM);

    // 3. Compute Differential Centering Error
    computeDifferentialCentering();
}

void SensorArray::computeDifferentialCentering() {
    if (readings_.wallLeft && readings_.wallRight) {
        // Ideal 2-wall corridor: error is difference between side walls
        readings_.centeringErrorMm = readings_.rightSideDistMm - readings_.leftSideDistMm;
    } else if (readings_.wallLeft) {
        // Only left wall visible: center against nominal left distance
        readings_.centeringErrorMm = 2.0f * (NOMINAL_SIDE_WALL_DIST_MM - readings_.leftSideDistMm);
    } else if (readings_.wallRight) {
        // Only right wall visible: center against nominal right distance
        readings_.centeringErrorMm = 2.0f * (readings_.rightSideDistMm - NOMINAL_SIDE_WALL_DIST_MM);
    } else {
        // No side walls (junction/open area): no centering correction
        readings_.centeringErrorMm = 0.0f;
    }
}

float SensorArray::rawAdcToDistanceMm(int adcVal) {
    if (adcVal <= 80) return 999.0f;
    // Characteristic 1/x voltage response for Sharp GP2Y0A51SK0F (calibrated for 3.3V ESP32 ADC)
    return 15000.0f / (static_cast<float>(adcVal) + 50.0f);
}

void SensorArray::clearTactileFlags() {
    readings_.leftTactileTriggered = false;
    readings_.rightTactileTriggered = false;
}
