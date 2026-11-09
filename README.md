# Smart-Multi-Cell-BMS
ESP32-based Smart Multi-Cell Battery Management System with relay protection, fault detection, Blynk IoT telemetry and analytics.
# Smart Multi-Cell Battery Management System

**Design and Implementation of a Smart Multi-Cell Battery Management System using ESP32, Wokwi Simulation and Blynk IoT**

An embedded systems and IoT project implementing a modular four-cell Battery Management System (BMS) prototype with battery monitoring, relay protection, supervisory fault management, event-driven telemetry and cloud-based analytics.

---

## 1. Project Overview

The Smart Multi-Cell BMS is an ESP32-based battery monitoring and protection demonstrator developed as part of a six-task internship project.

The system monitors four simulated cell-voltage inputs, calculates battery parameters, identifies abnormal operating conditions, controls a simulated protection relay, displays system information on an LCD and sends telemetry to the Blynk IoT dashboard.

The implementation is divided into six cooperating tasks:

1. Modular Battery Management Engine
2. Non-Blocking Protection Relay and Safety System
3. Flicker-Free LCD Display Engine
4. Fault State Machine with Structured Recovery
5. Event-Driven Telemetry and Live Blynk Dashboard
6. Enterprise Blynk Analytics and Decision Dashboard

**Project status:** Integrated six-task simulation prototype.

## 2. Technologies Used

| Component | Technology |
|---|---|
| Microcontroller | ESP32 DevKit C V4 |
| Programming language | C++ |
| Firmware framework | Arduino |
| Development environment | VS Code and PlatformIO |
| Hardware simulation | Wokwi |
| Cloud platform | Blynk IoT |
| Display | 16 × 2 I2C LCD |
| Communication | Wi-Fi and HTTPS |
| Concurrency | ESP32 FreeRTOS tasks |
| LCD library | LiquidCrystal_I2C |

## 3. System Architecture

The system follows a dual-core architecture.

**Core 1 – Control and Safety**

Responsible for:
- Battery voltage acquisition and analysis.
- Weakest and strongest cell identification.
- Voltage imbalance and trend calculations.
- Relay protection and safety logic.
- Supervisory fault-state management.
- LCD display updates.
- Serial Monitor reporting.

**Core 0 – Network and Telemetry**

Responsible for:
- Wi-Fi connection management.
- Event generation and queue management.
- HTTPS communication with Blynk Cloud.
- Telemetry transmission and retry handling.
- Blynk event logging.

A mutex-protected shared snapshot provides the data interface between the two cores.

## 4. Task-wise Implementation

### Task 1 – Modular Battery Management Engine

The battery engine uses a compile-time constant:

`constexpr uint8_t NUM_CELLS = 4;`

Battery-cell information is stored using:

`BatteryCell cells[NUM_CELLS]`

The `BMSSnapshot` structure exposes the calculated battery information through a common interface.

Implemented features:
- Individual cell-voltage monitoring.
- Cell percentage calculation.
- Average voltage and estimated battery SoC.
- Weakest and strongest cell identification.
- Minimum and maximum voltage calculation.
- Voltage imbalance calculation.
- Rising, falling and stable imbalance trend detection.
- SoC-dependent adaptive imbalance thresholds.

Adaptive thresholds:

| Estimated SoC | Imbalance threshold |
|---|---:|
| ≥ 80% | 0.05 V |
| 50% to below 80% | 0.08 V |
| 20% to below 50% | 0.10 V |
| Below 20% | 0.12 V |

The main battery-engine interface consists of `runBatteryEngine()` and `getSnapshot()`.

The software architecture supports expansion from four to sixteen cells using the same cell-processing logic. Physical sixteen-cell measurement would require an external ADC or suitable analogue multiplexer and a safe measurement front end.

### Task 2 – Non-Blocking Protection Relay and Safety System

The protection subsystem uses a four-state relay finite-state machine:

- CONNECTED
- TRIP_PENDING
- TRIPPED
- RECOVERING

Implemented features:
- Non-blocking timer-based transitions.
- 400 ms fault debounce.
- Weak-cell and high-voltage protection.
- Hysteresis to prevent relay chatter.
- Five-sample per-cell statistical history.
- Frozen-sensor detection.
- Out-of-range voltage detection.
- Unrealistic voltage-jump detection.
- Three-second timed recovery.
- Fault and relay transition logging.

Configured protection thresholds:

| Condition | Trip threshold | Release threshold |
|---|---:|---:|
| Weak cell | Below 30% | Above 35% |
| High cell | Above 90% | Below 85% |

### Task 3 – Flicker-Free LCD Display Engine

The system uses a 16 × 2 I2C LCD.

Three normal information pages are automatically displayed:

1. Battery page – battery status, SoC and adaptive threshold.
2. System page – relay state, supervisory state and cell information.
3. Telemetry page – average voltage, imbalance, trend and connectivity.

Implemented display features:
- Automatic page rotation every three seconds.
- Content refresh interval of 250 ms.
- Differential character rendering.
- No unnecessary full-screen clearing.
- Dedicated FAILSAFE and SHUTDOWN display overrides.
- Automatic return to normal display operation after successful recovery.

### Task 4 – Fault State Machine with Structured Recovery

A separate supervisory finite-state machine manages four operating states:

- NORMAL
- DEGRADED
- FAILSAFE
- SHUTDOWN

Fault sources are classified as:
- BATTERY
- ADC
- RELAY
- COMMUNICATION

The system identifies specific fault IDs, retains fault information and generates timestamped transition logs containing the previous state, new state, fault source and fault ID.

Implemented features:
- Relay command-feedback mismatch detection.
- ADC frozen and out-of-range fault classification.
- Communication-loss monitoring.
- Compound critical fault detection.
- FAILSAFE timeout and SHUTDOWN escalation.
- Latched SHUTDOWN state.
- Sequential recovery verification.

The recovery sequence checks:

`ADC → BATTERY → RELAY → COMMUNICATION`

Each stage must remain healthy for the configured one-second hold interval before the system advances.

### Task 5 – Event-Driven Telemetry and Live Blynk Dashboard

The telemetry system uses HTTPS communication with Blynk Cloud instead of the realtime Blynk library.

Events are generated for:
- System boot.
- Relay or supervisory state transitions.
- Significant battery parameter changes.
- Periodic heartbeat updates.
- Wi-Fi link loss and restoration.

Implemented features:
- Fixed-size 24-entry FIFO telemetry queue.
- Event sequence numbers and timestamps.
- Offline event retention in RAM.
- Oldest-event discard policy when the queue is full.
- Non-blocking Wi-Fi reconnection state machine.
- RSSI monitoring.
- HTTPS batch updates.
- Blynk device event logging.

Battery and system information is published through Blynk datastreams V0–V13, with additional Task 5 and Task 6 information through V14–V36.

**Verification qualification:** Queue accumulation and ordered replay were demonstrated using a temporary verification build. The final integrated firmware includes the queue and replay implementation, but a repeatable network-outage test on the final build remains to be documented.

### Task 6 – Enterprise Blynk Analytics and Decision Dashboard

The analytics layer provides a prototype risk indicator and rule-based maintenance recommendations.

The composite risk score ranges from 0 to 100 and considers:

| Risk component | Maximum contribution |
|---|---:|
| Voltage imbalance | 30 |
| State of Charge | 15 |
| Imbalance trend | 15 |
| Recent fault frequency | 15 |
| Supervisory system state | 25 |

Additional features:
- Rolling ten-minute fault-frequency tracking.
- Structured state-transition history.
- Human-readable maintenance recommendations.
- Battery and system-status monitoring.
- Uptime and telemetry information.
- Blynk historical trend visualisation.
- State severity representation.

The risk score is a demonstrator heuristic and is not a validated battery State of Health or remaining-useful-life prediction.

## 5. Hardware Connections

The Wokwi simulation uses four potentiometers to represent cell-voltage inputs.

| Component | ESP32 GPIO |
|---|---:|
| Cell 1 input | GPIO 34 |
| Cell 2 input | GPIO 35 |
| Cell 3 input | GPIO 32 |
| Cell 4 input | GPIO 33 |
| LCD SDA | GPIO 21 |
| LCD SCL | GPIO 22 |
| Relay output | GPIO 19 |
| Relay feedback | GPIO 27 |
| Red LED | GPIO 2 |
| Green LED | GPIO 4 |
| Yellow LED | GPIO 5 |
| Buzzer | GPIO 18 |

LCD I2C address: `0x27`

The relay-feedback switch simulates an auxiliary relay feedback contact.

The complete circuit configuration is provided in `diagram.json`.

## 6. Repository Structure

```text
Smart-Multi-Cell-BMS/
│
├── src/
│   └── main.cpp
│
├── docs/
│   ├── Project_Report.pdf
│   ├── Task_1_Verification.pdf
│   ├── Task_2_Verification.pdf
│   ├── Task_3_Verification.pdf
│   ├── Task_4_Verification.pdf
│   └── evidence/
│
├── README.md
├── platformio.ini
├── diagram.json
├── BUG_LOG.md
├── ASSUMPTIONS.md
├── SELF_ATTESTATION_CHECKLIST.md
└── .gitignore
```

Additional documentation and evidence files will be added as the repository is completed.

## 7. Setup and Build Instructions

### Requirements

- Visual Studio Code.
- PlatformIO IDE extension.
- ESP32 development environment.
- Wokwi simulator or Wokwi extension.
- LiquidCrystal_I2C library.

### PlatformIO configuration

The project uses the ESP32 development-board environment with the Arduino framework.

### Build firmware

Open the project directory in VS Code and execute:

```bash
pio run
```

### Upload firmware

Connect a compatible ESP32 development board and execute:

```bash
pio run -t upload
```

### Open Serial Monitor

```bash
pio device monitor
```

Serial communication is configured at 115200 baud.

## 8. Blynk Dashboard

The Blynk dashboard provides monitoring of:
- Average and individual cell voltages.
- Estimated battery SoC.
- Weakest and strongest cells.
- Voltage imbalance and adaptive threshold.
- Imbalance trend.
- Relay and supervisory states.
- Fault source and fault identifier.
- Wi-Fi RSSI and network condition.
- Offline queue depth.
- Event sequence and transmission status.
- Risk score and maintenance advice.
- Historical trends and state-transition events.

A custom Blynk event with the event code `bms_transition` is used for system-transition logging.

## 9. Verification and Test Results

Testing was conducted primarily through Wokwi simulation, Serial Monitor outputs, LCD observations and Blynk dashboard evidence.

| Task | Verification summary |
|---|---|
| Task 1 | Cell-extreme detection, imbalance trend and adaptive threshold tests documented as passed. Four-to-sixteen-cell scalability evaluated at the software architecture level. |
| Task 2 | Normal operation, weak/high-cell protection, debounce, hysteresis, anomaly detection and repeated fault-cycle tests documented as passed. |
| Task 3 | Page rotation, differential rendering, non-blocking display operation, fault override and recovery tests documented as passed. |
| Task 4 | Fault classification, relay mismatch and sequential recovery demonstrated. Some communication-fault tests used a temporary verification fixture. |
| Task 5 | Live telemetry and transition logging observed. Offline queue replay demonstrated with a temporary verification build. |
| Task 6 | Risk scoring, fault tracking, maintenance recommendations and dashboard history demonstrated. |

The detailed test procedures, screenshots and observations are provided in the project report and supporting verification documents.

## 10. Engineering Assumptions and Limitations

This project is a simulated learning and evaluation prototype.

The following limitations apply:

- Four potentiometers represent the cell-voltage inputs; no physical battery cells are connected.
- SoC is estimated using linear voltage normalisation and is not chemistry-specific.
- Adaptive imbalance thresholds depend on estimated SoC; discharge-rate compensation is not implemented.
- No active cell-balancing circuit is controlled.
- Sixteen-cell scalability is demonstrated at the software architecture level, not through physical sixteen-cell hardware.
- The relay and feedback contact are simulated.
- Offline event history is stored in RAM and does not survive a reset.
- The risk score is a heuristic rather than a validated battery-health prediction.
- Production deployment would require a suitable battery-monitoring front end, isolation, calibrated sensing, hardware protection and independent safety validation.

This prototype must not be used as the sole protection system for a real battery pack.

## 11. Project Documentation

The `docs/` directory contains the consolidated internship report and task-specific verification records.

The final documentation covers:
- Project objectives and architecture.
- Hardware and software implementation.
- Task-wise design and verification.
- State-machine operation.
- Serial Monitor and Blynk evidence.
- Challenges and engineering limitations.
- Future improvements.

## 12. Future Improvements

Potential improvements include:
- External ADC or battery-monitoring AFE for larger cell configurations.
- Chemistry-aware SoC estimation.
- Current and temperature sensing.
- Physical cell-balancing circuitry.
- Hardware contactor protection and pre-charge.
- Persistent event storage.
- Secure cloud communication and credential management.
- Hardware-in-the-loop testing and physical battery validation.

## 13. Project Links

**GitHub Repository:**  
https://github.com/darrenfernandes251-spec/Smart-Multi-Cell-BMS

**Wokwi Simulation:**  
To be added after publishing the final Wokwi project.

**Project Report:**  
[View Complete Internship Report](docs/Project_Report.pdf)

---

**Project:** Smart Multi-Cell Battery Management System  
**Submitted by:** Darren John Fernandes  
**Department:** Electronics and Communication Engineering  
**Institution:** Nitte Meenakshi Institute of Technology, Bengaluru  
**Internship Organisation:** Elevance Skills  
**Domain:** Embedded Systems / IoT
