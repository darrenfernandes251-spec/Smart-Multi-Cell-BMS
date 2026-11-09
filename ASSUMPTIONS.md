# Engineering Assumptions – Smart Multi-Cell Battery Management System

This document records the assumptions and design constraints considered during the implementation and verification of the Smart Multi-Cell Battery Management System.

## 1. Cell Voltage Simulation

- The prototype uses four potentiometers in Wokwi to represent four individual battery-cell voltage inputs.
- Each potentiometer provides a simulated analogue voltage signal to the ESP32.
- The ADC measurement range is treated as 0–3.3 V for the prototype.
- The current implementation uses four cell inputs, configured through the compile-time constant `NUM_CELLS`.

## 2. Battery State of Charge

- Cell percentage is calculated using linear voltage normalisation.
- Overall battery SoC is estimated from the average cell voltage.
- The estimation is intended for demonstration and is not based on a specific battery chemistry.
- Chemistry-specific voltage profiles and advanced SoC estimation algorithms are outside the current project scope.

## 3. Adaptive Imbalance Threshold

- The adaptive voltage-imbalance threshold is selected according to the estimated battery SoC.
- Four threshold bands are implemented: 0.05 V, 0.08 V, 0.10 V and 0.12 V.
- Discharge-rate-based compensation is not implemented because the current simulation does not include a current sensor or discharge-load model.

## 4. Hardware and Cell Scalability

- The four-cell Wokwi circuit represents the battery monitoring functionality.
- Expansion to sixteen cells is considered at the software architecture level using parameterised arrays and processing loops.
- The memory estimate assumes each `BatteryCell` structure occupies approximately eight bytes.
- Physical sixteen-cell measurement has not been demonstrated.
- A larger physical implementation would require an appropriate external ADC or analogue multiplexer and a safe measurement front end.

## 5. Relay and Feedback Simulation

- The relay output is controlled through ESP32 GPIO 19.
- GPIO 27 represents the relay auxiliary feedback contact.
- A slide switch is used to simulate the feedback signal.
- The simulated feedback arrangement is not equivalent to a complete physical contactor monitoring system.

## 6. Protection Logic

- Weak-cell and high-voltage thresholds are based on the configured percentage limits.
- Fault debounce, hysteresis and recovery timings use the values configured in the firmware.
- Sensor anomaly detection uses a five-sample history window and statistical comparisons.
- The frozen-sensor detector evaluates individual cell activity relative to the activity of the other cells.

## 7. Display

- The LCD is a 16 × 2 I2C character display.
- Its I2C address is assumed to be 0x27.
- The display refresh interval is 250 ms.
- Normal information pages rotate at three-second intervals.
- Critical fault and shutdown screens override normal page rotation.

## 8. Networking and Telemetry

- Wi-Fi is used as the communication medium for Blynk Cloud.
- Network connectivity and Blynk HTTP transaction success are monitored separately.
- The telemetry queue has a fixed capacity of 24 events.
- The queue is stored in RAM and does not survive ESP32 resets or power loss.
- The oldest event is discarded when the queue reaches capacity.
- Wi-Fi retry and telemetry transmission use configured non-blocking timing intervals.

## 9. Analytics

- The risk score is a prototype heuristic ranging from 0 to 100.
- It combines voltage imbalance, estimated SoC, imbalance trend, recent fault frequency and supervisory system state.
- Maintenance recommendations are generated using predefined rules.
- The risk score is not a validated State of Health, remaining-useful-life or safety prediction model.

## 10. Prototype and Safety Limitations

- The project is developed and verified primarily through Wokwi simulation.
- No actual battery pack or high-voltage cell measurement system is connected.
- Active cell balancing is not implemented.
- Physical battery protection hardware and independent safety validation are outside the current prototype scope.
- The system is intended for educational demonstration and must not be used as the sole protection mechanism for a real battery pack.

---

**Project:** Smart Multi-Cell Battery Management System  
**Platform:** ESP32, Wokwi, PlatformIO and Blynk IoT  
**Domain:** Embedded Systems / IoT
