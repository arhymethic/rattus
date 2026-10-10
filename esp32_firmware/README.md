# Autonomous 10x10 Micromouse: ESP32 Firmware Package

A production-ready firmware package configured for an **ESP32** microcontroller, ready to flash directly onto your autonomous micromouse robot.

---

## 1. Sensor Selection Guide: What Sensors to Use

To achieve high-speed navigation, zero wall collisions, and fast braking, use this **hybrid 4-sensor array**:

### A. Dual Front Time-of-Flight Sensors (Longitudinal Braking & Angular Alignment)
* **Recommended Sensor:** **VL53L1X** (STMicroelectronics I2C ToF breakout, e.g. Pololu #3415 or CJMCU-531).
* **Quantity:** **2 units** (Left Front & Right Front).
* **Placement & Orientation:**
  - Mount both facing straight forward, spaced **40 mm to 55 mm apart**.
  - Position them **15 mm to 25 mm above the maze floor** (well below the 100 mm wall height).
* **Why 2 Front Sensors:**
  1. **Long-Distance Braking:** Detects upcoming walls up to 1.3 meters ahead, allowing the robot to sprint straight corridors at high velocity and brake smoothly into cell centers.
  2. **Angular Skew Correction:** If the robot approaches a front wall slightly crooked, `frontLeftToF - frontRightToF` detects the angular error and straightens the robot before turning!
* **I2C Address Assignment:**
  - Both sensors boot on I2C address `0x29`.
  - The firmware uses their **XSHUT pins** to release and reassign:
    * Left ToF $\rightarrow$ **`0x30`**
    * Right ToF $\rightarrow$ **`0x31`**

### B. Dual Side Analog Distance Sensors (Continuous Wall-Centering Feedback)
* **Recommended Sensor Option 1 (Plug & Play):** **Sharp GP2Y0A51SK0F** (Analog IR distance sensor, 2 cm to 15 cm measuring range).
* **Recommended Sensor Option 2 (Custom/Pro):** 940 nm High-power IR Emitters (e.g. Everlight) + Phototransistors/Photodiodes (e.g. Vishay TEFT4300) with a modulation NPN/MOSFET.
* **Quantity:** **2 units** (Left Side & Right Side).
* **Placement & Orientation:**
  - Mount perpendicular or angled **45° forward-outward** on the sides of the chassis.
* **Why Side Analog IR:**
  - Analog ADC reads execute in **microseconds** (unlike I2C which takes milliseconds).
  - Feeds the **100 Hz differential steering PID loop** (`rightDist - leftDist = 0`) to keep the robot locked to the corridor centerline without scraping walls.

### C. Tactile Bumpers (Physical Rule Compliance & Failsafe)
* **Recommended Switch:** Subminiature snap-action microswitches (e.g., **Omron D2F-01L**) or carbon-fiber leaf switches.
* Connected directly to ESP32 internal pull-up GPIOs with hardware interrupts.

---

## 2. ESP32 Pin Assignment Table

All pin variables are defined in [`include/Config.h`](include/Config.h). You can change any pin to match your physical schematic:

| Variable Name | Default Pin | Direction | Function / Description |
|---|---|---|---|
| **`PIN_MOTOR_LEFT_PWM`** | **`GPIO 25`** | Output | Left Motor Speed (LEDC PWM Channel 0) |
| **`PIN_MOTOR_LEFT_DIR`** | **`GPIO 26`** | Output | Left Motor Direction (HIGH=Forward, LOW=Reverse) |
| **`PIN_MOTOR_RIGHT_PWM`** | **`GPIO 27`** | Output | Right Motor Speed (LEDC PWM Channel 1) |
| **`PIN_MOTOR_RIGHT_DIR`** | **`GPIO 14`** | Output | Right Motor Direction (HIGH=Forward, LOW=Reverse) |
| **`PIN_ENCODER_LEFT_A`** | **`GPIO 34`** | Input | Left Wheel Quadrature Encoder Phase A (Interrupt) |
| **`PIN_ENCODER_LEFT_B`** | **`GPIO 35`** | Input | Left Wheel Quadrature Encoder Phase B |
| **`PIN_ENCODER_RIGHT_A`**| **`GPIO 36`** | Input | Right Wheel Quadrature Encoder Phase A (Interrupt) |
| **`PIN_ENCODER_RIGHT_B`**| **`GPIO 39`** | Input | Right Wheel Quadrature Encoder Phase B |
| **`PIN_IR_LEFT_ADC`** | **`GPIO 32`** | Analog In | Left Side Analog IR (ESP32 ADC1_CH4) |
| **`PIN_IR_RIGHT_ADC`** | **`GPIO 33`** | Analog In | Right Side Analog IR (ESP32 ADC1_CH5) |
| **`PIN_I2C_SDA`** | **`GPIO 21`** | I2C Data | Dual VL53L1X Shared I2C Bus Data |
| **`PIN_I2C_SCL`** | **`GPIO 22`** | I2C Clock | Dual VL53L1X Shared I2C Bus Clock |
| **`PIN_TOF_LEFT_XSHUT`** | **`GPIO 18`** | Output | Left VL53L1X Shutdown Pin (For I2C address 0x30) |
| **`PIN_TOF_RIGHT_XSHUT`**| **`GPIO 19`** | Output | Right VL53L1X Shutdown Pin (For I2C address 0x31) |
| **`PIN_BUMPER_LEFT`** | **`GPIO 4`** | Input Pullup | Left Tactile Emergency Collision Bumper |
| **`PIN_BUMPER_RIGHT`** | **`GPIO 5`** | Input Pullup | Right Tactile Emergency Collision Bumper |
| **`PIN_START_BUTTON`** | **`GPIO 23`** | Input Pullup | Physical Pushbutton to Start Run |
| **`PIN_STATUS_LED`** | **`GPIO 2`** | Output | Diagnostic Status LED |
| **`PIN_BUZZER`** | **`GPIO 12`** | Output | Piezo Buzzer for Audio Telemetry |

> [!NOTE]
> `GPIO 34, 35, 36, 39` on the ESP32 are **input-only pins** with no internal pull-ups, making them ideal for receiving high-speed encoder pulses.
> `GPIO 32, 33` belong to **ADC1**, which is completely unaffected by Wi-Fi or Bluetooth usage.

---

## 3. Recommended Actuation & Power Components

1. **Motors:** N20 Micro Metal Gearmotors (6V, 1:30 or 1:50 gear ratio) with integrated magnetic quadrature encoders (12 CPR $\times$ 30 gear ratio $\approx$ 360 ticks/rev).
2. **Motor Driver:** **TB6612FNG** or **DRV8833** Dual H-Bridge module (handles up to 1.2A per motor).
3. **Battery:** **2S LiPo (7.4V, 300–450 mAh, 30C–45C)** with a high-efficiency 5V / 3.3V switching buck regulator (e.g. MP1584EN or LM2596) powering the ESP32 and sensors.
4. **Power Switch:** Physical heavy-duty slide or toggle switch cutting the LiPo battery positive lead directly before the buck converters.

---

## 4. How to Flash the Firmware

### Option A: Using PlatformIO (Recommended)
1. Install [PlatformIO Core](https://platformio.org/) or the PlatformIO extension in VSCode.
2. Connect your ESP32 board via USB.
3. Open a terminal in this folder (`esp32_firmware`):
   ```bash
   cd esp32_firmware
   pio run -t upload
   ```
4. Open the serial monitor:
   ```bash
   pio device monitor -b 115200
   ```

### Option B: Using Arduino IDE
1. Open the Arduino IDE.
2. Go to **Tools $\rightarrow$ Board** and select **ESP32 Dev Module**.
3. Go to **Sketch $\rightarrow$ Include Library $\rightarrow$ Manage Libraries...** and install:
   - **`VL53L1X` by Pololu**
4. Copy the files into an Arduino sketch folder and hit **Upload**.

---

## 5. Robot Calibration Checklist Before First Run

1. **Encoder Calibration ([`include/Config.h`](include/Config.h)):**
   - Measure your wheel diameter (default: `32.0 mm`).
   - Check your encoder ticks per revolution (default: `360.0 ticks`).
   - Measure your wheel track width between tire centerlines (default: `74.0 mm`).
2. **Side IR ADC Calibration ([`src/Sensors.cpp`](src/Sensors.cpp)):**
   - Place the robot centered in a 192 mm corridor.
   - Verify that `leftSideDistMm` and `rightSideDistMm` read $\approx 55 \text{ mm}$.
   - If using custom IR photodiodes, adjust the curve in `rawAdcToDistanceMm()`.
3. **Motor Direction Test:**
   - Verify that positive PWM drives the wheels forward. If a motor spins backward, swap the corresponding `PIN_MOTOR_..._DIR` or motor leads.
