# Autonomous Micromouse: Simulation & ESP32 Firmware

A championship-grade C++ firmware architecture and simulation environment for an autonomous Micromouse robot. Interfaces seamlessly with **Mackorone's Micromouse Simulator (MMS)** and deploys to an **ESP32** microcontroller via **PlatformIO**.

---

## 1. Competition & Maze Constraints

| Parameter | Value | Details |
|---|---|---|
| **Grid Dimensions** | **10 x 10** | Standard 10x10 competition grid (100 cells) |
| **Cell Pitch** | **198 mm** | Center-to-center cell spacing |
| **Path Width** | **192 mm** | Corridor width between opposing walls |
| **Wall Height** | **100 mm** | Barrier height (6 mm thickness) |
| **Robot Max Envelope**| **190 x 190 mm** | Bounding box limit |
| **Robot Max Height**  | **< 90 mm** | Strictly under 100 mm wall height |
| **Start Cell** | `(0, 0)` facing **North** | Standard competition starting configuration |
| **Center Destination**| `(4, 4), (4, 5), (5, 4), (5, 5)` | 2x2 central destination target zone |
| **Autonomy** | Fully autonomous | No external positioning, remote control, or manual aid |
| **Run Structure** | 3-Phase Engine | Search Run $\rightarrow$ Return Run $\rightarrow$ Speed Run |

---

## 2. 3-Phase Championship Navigation Engine

1. **Phase 1: Search Run (Exploration):**
   - Navigates from `(0,0)` to the center goal using IEEE UCI BFS Flood Fill.
   - Discovers and maps walls in real-time using dual front ToF + dual side IR sensors.
   - Renders dynamic distance fields and marks visited corridors in Cyan (`C`).
2. **Phase 2: Return Run (Corridor Expansion):**
   - Automatically re-targets origin `(0,0)` via `solver.setGoalsStart()`.
   - Navigates back while mapping alternative open corridors in Blue (`B`).
   - Aligns facing North at `(0,0)` ready for sprint.
3. **Phase 3: Speed Run (Pre-computed Sprint):**
   - Floods strictly through verified visited cells (`floodKnownOnly()`) ensuring zero collisions with unmapped walls.
   - Pre-computes optimal gradient and merges consecutive straight cells into multi-cell bursts (`moveForward(N)`).
   - Sprints to the center in record time.

---

## 3. Codebase Organization

```
.
├── CMakeLists.txt             # Native CMake build configuration
├── Makefile                   # High-speed compilation & test runner
├── platformio.ini             # PlatformIO config (native 'sim' & 'esp32dev')
├── include/
│   ├── API.h                  # Mackorone Micromouse Simulator (MMS) bridge
│   ├── Config.h               # Maze constants, physical limits, PID gains, pins
│   ├── FloodFill.h            # Static ring-buffer BFS solver & speed-run planner
│   ├── Motors.h               # Motor controller HAL, discrete PID, encoder odometry
│   └── Sensors.h              # Hybrid sensor HAL (Analog IR + ToF + EMA filter)
├── src/
│   ├── API.cpp                # MMS stdin/stdout protocol implementation
│   ├── FloodFill.cpp          # Static BFS flood fill, visual sync, path merger
│   ├── Motors.cpp             # PID computation, differential drive kinematics
│   ├── Sensors.cpp            # Differential centering computation & EMA filter
│   └── main.cpp               # 3-Phase state machine & ESP32 / Sim entry points
└── test/
    ├── run_all_tests.py       # Unified test runner (`make test`)
    ├── test_mms_protocol.py   # Protocol compliance unit test (10x10)
    ├── test_screenshot_trap.py# Dead-end wall trap integration test (10x10)
    ├── test_championship.py   # Full 3-phase championship test (10x10)
    ├── test_10x10_competition.py # 10x10 competition benchmark test
    └── maze10x10.num          # Benchmark 10x10 maze layout
```

---

## 4. Building & Running in Simulation (MMS)

### Quick Start
```bash
# Compile native simulator binary (sim_mouse)
make

# Run automated test suite
make test

# Clean build artifacts
make clean
```

### Configuring Mackorone's Micromouse Simulator
1. Launch **MMS** (`./bin/mms`).
2. Click **"+"** in the top-right Mouse panel.
3. Enter configuration:
   - **Name:** `ESP32-Mouse`
   - **Directory:** `/home/rhythm/Projects/mouse`
   - **Build Command:** `make`
   - **Run Command:** `./sim_mouse`
4. Select your preferred maze in the top Config panel (e.g. `example5.num`).
5. Click **Build**, then click **Run**.

---

## 5. ESP32 Physical Hardware Firmware

The same codebase compiles natively for the **ESP32** microcontroller using PlatformIO:

```bash
# Build firmware for ESP32 target
pio run -e esp32dev

# Upload firmware to connected ESP32 board
pio run -e esp32dev -t upload
```

- **Dual-Core Architecture:** Core 0 runs the 1 kHz motor PID & hardware PCNT encoder loop; Core 1 runs sensor reading and flood fill.
- **Hybrid Sensor Processing:** Exponential Moving Average (EMA, $\alpha=0.35$) filtering eliminates analog ADC noise from side IR sensors.
- **Zero Heap Allocations:** All BFS queues, distance matrices, and path arrays use static memory buffers, eliminating stack overflow and fragmentation risks.
