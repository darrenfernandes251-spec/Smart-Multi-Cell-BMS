# Bug Log – Smart Multi-Cell Battery Management System

This document records the major implementation challenges, observed issues, debugging activities and resolutions encountered during the development and integration of the six-task BMS project.

## 1. Wokwi Network Interruption Testing

**Issue:**  
Wokwi maintained the Wi-Fi/Blynk connection during simulation, making repeatable communication-fault and network-outage testing difficult.

**Resolution:**  
Temporary verification controls were introduced to simulate communication failures and test offline event accumulation and recovery. These controls were subsequently removed from the final integrated firmware.

**Status:** Implemented network recovery logic; final-build outage verification remains to be repeated.

## 2. Blynk Library and Serial Communication Integration

**Issue:**  
The realtime Blynk library and its communication/debugging activity created integration difficulties, including Serial Monitor output problems.

**Resolution:**  
The implementation was changed to use Blynk Cloud HTTPS endpoints through HTTPClient and WiFiClientSecure. Network operations were moved into a separate FreeRTOS task on Core 0, while battery monitoring and protection remained on Core 1.

**Status:** Addressed in the final implementation.

## 3. Blynk Timeline HTTP 400 Error

**Issue:**  
The Blynk transition-event logging endpoint initially returned HTTP 400 because the required custom event had not been configured in the Blynk template.

**Resolution:**  
A custom event named BMS State Transition was configured with the event code `bms_transition`. Device-page visibility was enabled.

**Status:** Resolved; successful transition-event logging was subsequently observed.

## 4. Blynk Historical Charts Showing No Data

**Issue:**  
Historical dashboard charts initially displayed no data because sufficient historical samples had not yet accumulated.

**Resolution:**  
History-enabled datastreams and periodic heartbeat telemetry were used to accumulate samples. The dashboard charts were subsequently populated.

**Status:** Addressed; chart population depends on incoming samples and History configuration.

## 5. Duplicate Fault Counting During State Escalation

**Issue:**  
A fault progressing from FAILSAFE to SHUTDOWN could be interpreted as an additional fault occurrence even when the underlying fault had not changed.

**Resolution:**  
Fault counting was revised to increment only when a new non-NONE fault identifier or fault source appears. Escalation of the same fault is not counted as a separate occurrence.

**Status:** Addressed in the integrated firmware.

## 6. ESP32 Analogue Input Scalability

**Issue:**  
The number of directly connected analogue inputs available for cell monitoring is limited by the ESP32 hardware configuration.

**Resolution:**  
The software architecture uses `NUM_CELLS` and parameterised processing loops. Larger physical configurations will require suitable external ADC hardware or an analogue multiplexer and an appropriate measurement front end.

**Status:** Software scalability addressed; sixteen-cell hardware implementation remains future work.

## 7. Significant Battery-Change Event Verification

**Issue:**  
The significant-parameter-change telemetry test did not consistently demonstrate the expected threshold-crossing event during slow manual potentiometer adjustments.

**Resolution:**  
The event-generation thresholds and logic are present in the final firmware. Additional controlled testing is required to conclusively verify this trigger.

**Status:** Implementation present; verification inconclusive.

## 8. Cross-Core Data Sharing

**Issue:**  
Battery monitoring and network telemetry execute on separate ESP32 cores, requiring safe sharing of measurement and system-state information.

**Resolution:**  
A mutex-protected snapshot interface was implemented. The control-side task uses a zero-wait mutex acquisition when publishing telemetry data, avoiding unnecessary blocking of the safety loop.

**Status:** Implemented.

## 9. Final Integration and Verification

The integrated firmware combines battery analysis, relay protection, LCD rendering, supervisory fault management, telemetry and dashboard analytics.

A final controlled relay-mismatch test demonstrated the supervisory transition from NORMAL to FAILSAFE, followed by sequential recovery verification and return to NORMAL.

The complete verification evidence and remaining test qualifications are documented in the consolidated internship project report.

---

**Project:** Smart Multi-Cell Battery Management System  
**Domain:** Embedded Systems / IoT  
**Platform:** ESP32, PlatformIO, Wokwi and Blynk IoT
