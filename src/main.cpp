
#define ENABLE_NETWORKING 1


#define BLYNK_AUTH_TOKEN "YOUR_AUTH_TOKEN"


#include <Wire.h>
#include <string.h>
#include <math.h>   // strcpy() for the fixed-size char[] fields in BMSSnapshot
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
char ssid[] = "Wokwi-GUEST";
char pass[] = "";
LiquidCrystal_I2C lcd(0x27, 16, 2);

constexpr uint8_t NUM_CELLS = 4;

const uint8_t CELL_PINS[NUM_CELLS] =
{
  34, 35, 32, 33
};


#define RED_LED       2
#define GREEN_LED     4
#define YELLOW_LED    5
#define BUZZER        18
#define RELAY         19

#define RELAY_FB      27




struct BatteryCell
{
  float voltage;
  int   percent;
};

struct BMSSnapshot
{
  uint8_t cellCount;
  BatteryCell cells[NUM_CELLS];

  float averageVoltage;
  float batterySoC;

  uint8_t weakestCell;
  uint8_t strongestCell;
  float minimumVoltage;
  float maximumVoltage;
  float voltageImbalance;

  char imbalanceTrend[8];   // "START"/"RISING"/"FALLING"/"STABLE" - fixed size, no heap
  float adaptiveThreshold;

  char status[12];          // "NORMAL"/"WEAK CELL"/"HIGH VOLT"/"IMBALANCE" - fixed size, no heap
};


BMSSnapshot bms;


BMSSnapshot bmsShared;

float previousImbalance = 0.0;
bool previousDataAvailable = false;
constexpr float TREND_TOLERANCE = 0.005;



SemaphoreHandle_t bmsMutex;

volatile bool g_wifiConnected  = false;
volatile bool g_blynkConnected = false;


// ============================================================
// FUNCTION: PUBLISH bms -> bmsShared (core 1 calls this)
// ============================================================

void publishSharedSnapshot()
{
  
  if (xSemaphoreTake(bmsMutex, 0) == pdTRUE)
  {
    bmsShared = bms;
    xSemaphoreGive(bmsMutex);
  }
}




bool copySharedSnapshot(BMSSnapshot& out)
{
  if (xSemaphoreTake(bmsMutex, 0) == pdTRUE)
  {
    out = bmsShared;
    xSemaphoreGive(bmsMutex);
    return true;
  }
  return false;
}




constexpr int TRIP_LOW_PERCENT     = 30;
constexpr int RELEASE_LOW_PERCENT  = 35;
constexpr int TRIP_HIGH_PERCENT    = 90;
constexpr int RELEASE_HIGH_PERCENT = 85;

constexpr uint8_t WINDOW_SIZE      = 5;
constexpr float   FROZEN_EPSILON   = 0.002;

constexpr float ACTIVITY_STDDEV_MIN = 0.015;

constexpr float JUMP_ABS_MIN     = 0.30;
constexpr float JUMP_STDDEV_MULT = 6.0;
constexpr float MIN_STDDEV_FLOOR = 0.02;

constexpr float OUT_OF_RANGE_LOW  = 0.02;
constexpr float OUT_OF_RANGE_HIGH = 3.28;

constexpr unsigned long DEBOUNCE_MS = 400;
constexpr unsigned long RECOVERY_MS = 3000;


// ============================================================
// TASK 2 - PER-CELL HISTORY (core 1 only, unchanged)
// ============================================================

float cellHistory[NUM_CELLS][WINDOW_SIZE];
uint8_t cellHistoryCount[NUM_CELLS] = {0};
uint8_t cellHistoryIndex[NUM_CELLS] = {0};

bool cellWeakLatched[NUM_CELLS] = {false};
bool cellHighLatched[NUM_CELLS] = {false};


unsigned long lastActiveTime[NUM_CELLS] = {0};
constexpr unsigned long OBSERVATION_WINDOW_MS = 4000;



enum RelayState
{
  RELAY_CONNECTED,
  RELAY_TRIP_PENDING,
  RELAY_TRIPPED,
  RELAY_RECOVERING
};

RelayState relayState = RELAY_CONNECTED;

unsigned long debounceStartTime = 0;
unsigned long recoveryStartTime = 0;

bool rawFaultDetected = false;
String faultReason = "NONE";        // full detail, for Serial logging
String faultReasonShort = "NONE";   // compact (<=16 char) form, for the LCD fault page

enum FaultSource
{
  FAULT_SRC_NONE,
  FAULT_SRC_BATTERY,
  FAULT_SRC_ADC,
  FAULT_SRC_RELAY,
  FAULT_SRC_COMMUNICATION
};

enum FaultID
{
  FAULT_NONE,
  FAULT_BATTERY_WEAK,
  FAULT_BATTERY_HIGH,
  FAULT_ADC_FROZEN,
  FAULT_ADC_OUT_OF_RANGE,
  FAULT_ADC_JUMP,
  FAULT_RELAY_MISMATCH,
  FAULT_COMMUNICATION
};

const char* faultSourceName(FaultSource f)
{
  switch (f)
  {
    case FAULT_SRC_NONE:          return "NONE";
    case FAULT_SRC_BATTERY:       return "BATTERY";
    case FAULT_SRC_ADC:           return "ADC";
    case FAULT_SRC_RELAY:         return "RELAY";
    case FAULT_SRC_COMMUNICATION: return "COMMUNICATION";
  }
  return "UNKNOWN";
}

const char* faultIdName(FaultID id)
{
  switch (id)
  {
    case FAULT_NONE:            return "NONE";
    case FAULT_BATTERY_WEAK:    return "BATTERY_WEAK";
    case FAULT_BATTERY_HIGH:    return "BATTERY_HIGH";
    case FAULT_ADC_FROZEN:      return "ADC_FROZEN";
    case FAULT_ADC_OUT_OF_RANGE:return "ADC_OUT_OF_RANGE";
    case FAULT_ADC_JUMP:        return "ADC_JUMP";
    case FAULT_RELAY_MISMATCH:  return "RELAY_MISMATCH";
    case FAULT_COMMUNICATION:   return "COMMUNICATION";
  }
  return "UNKNOWN";
}

String faultIdShort(FaultID id)
{
  switch (id)
  {
    case FAULT_NONE:             return "NONE";
    case FAULT_BATTERY_WEAK:     return "BATT WEAK";
    case FAULT_BATTERY_HIGH:     return "BATT HIGH";
    case FAULT_ADC_FROZEN:       return "ADC FROZEN";
    case FAULT_ADC_OUT_OF_RANGE: return "ADC RANGE";
    case FAULT_ADC_JUMP:         return "ADC JUMP";
    case FAULT_RELAY_MISMATCH:   return "RELAY MISMATCH";
    case FAULT_COMMUNICATION:    return "COMM LOSS";
  }
  return "UNKNOWN";
}

FaultSource task2FaultSource = FAULT_SRC_NONE;   // classification of evaluateSafety()'s current rawFaultDetected
FaultID     task2FaultId     = FAULT_NONE;

// ------------------------------------------------------------
// TASK 3 - explicit LCD fault latch (preserved, still set/cleared
// by Task 2's relay FSM exactly as before). updateLCD() itself now
// keys off Task 4's systemState for page selection (see section 18
// of the spec) - this flag is kept because Task 2's own logging
// still relies on it, but it's no longer what the LCD override
// decision reads.
// ------------------------------------------------------------
bool lcdFaultLatched = false;


// ============================================================
// FUNCTION: STATE NAME
// ============================================================

const char* relayStateName(RelayState s)
{
  switch (s)
  {
    case RELAY_CONNECTED:    return "CONNECTED";
    case RELAY_TRIP_PENDING: return "TRIP_PENDING";
    case RELAY_TRIPPED:      return "TRIPPED";
    case RELAY_RECOVERING:   return "RECOVERING";
  }
  return "UNKNOWN";
}


// ============================================================
// FUNCTION: LOG A RELAY FSM TRANSITION
// ============================================================

void logTransition(RelayState from, RelayState to, const String& reason)
{
  Serial.print("[RELAY LOG] t=");
  Serial.print(millis());
  Serial.print("ms | ");
  Serial.print(relayStateName(from));
  Serial.print(" -> ");
  Serial.print(relayStateName(to));
  Serial.print(" | reason: ");
  Serial.println(reason);
}


// ============================================================
// FUNCTION: PUSH A READING INTO A CELL'S HISTORY WINDOW
// ============================================================

void pushCellHistory(uint8_t i, float voltage)
{
  cellHistory[i][cellHistoryIndex[i]] = voltage;
  cellHistoryIndex[i] = (cellHistoryIndex[i] + 1) % WINDOW_SIZE;

  if (cellHistoryCount[i] < WINDOW_SIZE)
  {
    cellHistoryCount[i]++;
  }
}


// ============================================================
// FUNCTION: MEAN AND STD DEV OF A CELL'S WINDOW
// ============================================================

void computeMeanStdDev(uint8_t i, float& mean, float& stdDev)
{
  uint8_t n = cellHistoryCount[i];

  if (n == 0)
  {
    mean = 0.0;
    stdDev = 0.0;
    return;
  }

  float sum = 0.0;
  for (uint8_t k = 0; k < n; k++)
  {
    sum += cellHistory[i][k];
  }
  mean = sum / n;

  float variance = 0.0;
  for (uint8_t k = 0; k < n; k++)
  {
    float diff = cellHistory[i][k] - mean;
    variance += diff * diff;
  }
  variance /= n;

  stdDev = sqrt(variance);
}


// ============================================================
// FUNCTION: IS THIS CELL'S WINDOW FLAT?
// ============================================================

bool isCellWindowFlat(uint8_t i)
{
  if (cellHistoryCount[i] < WINDOW_SIZE) return false;

  float minV = cellHistory[i][0];
  float maxV = cellHistory[i][0];

  for (uint8_t k = 1; k < WINDOW_SIZE; k++)
  {
    if (cellHistory[i][k] < minV) minV = cellHistory[i][k];
    if (cellHistory[i][k] > maxV) maxV = cellHistory[i][k];
  }

  return (maxV - minV) < FROZEN_EPSILON;
}


// ============================================================
// FUNCTION: IS THIS READING AN UNREALISTIC JUMP?
// ============================================================

bool isCellJump(uint8_t i, float newVoltage, float mean, float stdDev)
{
  if (cellHistoryCount[i] < WINDOW_SIZE) return false;

  float deviation = fabs(newVoltage - mean);
  float effectiveStdDev = (stdDev < MIN_STDDEV_FLOOR) ? MIN_STDDEV_FLOOR : stdDev;

  bool statisticalOutlier = deviation > (JUMP_STDDEV_MULT * effectiveStdDev);
  bool exceedsAbsoluteFloor = deviation > JUMP_ABS_MIN;

  return statisticalOutlier && exceedsAbsoluteFloor;
}


// ============================================================
// TASK 2 - EVALUATE SAFETY CONDITIONS (core 1 only, no mutex)
// Unchanged except each fault assignment now also tags
// task2FaultSource/task2FaultId for Task 4 to consume.
// ============================================================

// ------------------------------------------------------------
// Edge-trigger cache for fault message text (see note below).
// ------------------------------------------------------------
FaultID lastReportedFaultId = FAULT_NONE;
int     lastReportedFaultCell = -1;

void evaluateSafety()
{
  rawFaultDetected = false;
  task2FaultSource = FAULT_SRC_NONE;
  task2FaultId = FAULT_NONE;

  // faultCellIndex/faultDetailA/faultDetailB are the CHEAP numeric
  // detection result (no heap allocation) - computed fresh every
  // cycle as before. The expensive part - building faultReason/
  // faultReasonShort via String concatenation - only happens later,
  // and only when the winning fault actually CHANGES (see below).
  //
  // WHY THIS MATTERS: loop() has no delay(), so it runs thousands
  // of times per second. The previous version rebuilt faultReason
  // via String "+" concatenation INSIDE each pass, every single
  // cycle a fault condition stayed true - not just when it first
  // appeared. A continuously-active fault (e.g. a cell parked at
  // 0V, reading OUT_OF_RANGE nonstop) meant thousands of heap
  // allocations per second, nonstop, which fragments/exhausts the
  // ESP32's small heap and eventually hangs the device silently -
  // exactly the "works briefly, then the Serial Monitor goes dead"
  // symptom. Building the message only on the edge (fault newly
  // appears or changes) removes that churn entirely while leaving
  // every threshold, priority order, and transition unchanged.
  uint8_t faultCellIndex = 255;
  float faultDetailA = 0.0;
  float faultDetailB = 0.0;

  unsigned long now = millis();

  float cellMean[NUM_CELLS];
  float cellStdDev[NUM_CELLS];
  bool  cellActiveNow[NUM_CELLS];
  bool  cellRecentlyActive[NUM_CELLS];
  bool  cellFlat[NUM_CELLS];

  // Pass 0: window statistics for every cell, pre-push.
  for (uint8_t i = 0; i < bms.cellCount; i++)
  {
    computeMeanStdDev(i, cellMean[i], cellStdDev[i]);

    cellActiveNow[i] = (cellHistoryCount[i] >= WINDOW_SIZE) &&
                        (cellStdDev[i] > ACTIVITY_STDDEV_MIN);

    if (cellActiveNow[i])
    {
      lastActiveTime[i] = now;
    }

    cellRecentlyActive[i] = (lastActiveTime[i] != 0) &&
                             (now - lastActiveTime[i] <= OBSERVATION_WINDOW_MS);

    cellFlat[i] = isCellWindowFlat(i);
  }

  // Pass 0b: hysteresis latches update for EVERY cell every cycle.
  for (uint8_t i = 0; i < bms.cellCount; i++)
  {
    int p = bms.cells[i].percent;

    if (!cellWeakLatched[i] && p < TRIP_LOW_PERCENT)        cellWeakLatched[i] = true;
    else if (cellWeakLatched[i] && p > RELEASE_LOW_PERCENT) cellWeakLatched[i] = false;

    if (!cellHighLatched[i] && p > TRIP_HIGH_PERCENT)         cellHighLatched[i] = true;
    else if (cellHighLatched[i] && p < RELEASE_HIGH_PERCENT)  cellHighLatched[i] = false;
  }

  // Priority Pass 1: OUT_OF_RANGE
  for (uint8_t i = 0; i < bms.cellCount && !rawFaultDetected; i++)
  {
    float v = bms.cells[i].voltage;
    if (v < OUT_OF_RANGE_LOW || v > OUT_OF_RANGE_HIGH)
    {
      rawFaultDetected = true;
      task2FaultSource = FAULT_SRC_ADC;
      task2FaultId = FAULT_ADC_OUT_OF_RANGE;
      faultCellIndex = i;
      faultDetailA = v;
    }
  }

  // Priority Pass 2: FROZEN (sequential/broader-window activity check)
  for (uint8_t i = 0; i < bms.cellCount && !rawFaultDetected; i++)
  {
    uint8_t activeOthers = 0;
    for (uint8_t j = 0; j < bms.cellCount; j++)
    {
      if (j != i && cellRecentlyActive[j]) activeOthers++;
    }
    uint8_t otherCount = bms.cellCount - 1;
    bool majorityActiveElsewhere = (otherCount > 0) && (activeOthers * 2 > otherCount);

    bool thisCellTrulyFlat = cellFlat[i] && !cellRecentlyActive[i];

    if (thisCellTrulyFlat && majorityActiveElsewhere)
    {
      rawFaultDetected = true;
      task2FaultSource = FAULT_SRC_ADC;
      task2FaultId = FAULT_ADC_FROZEN;
      faultCellIndex = i;
    }
  }

  // Priority Pass 3: UNREALISTIC JUMP
  for (uint8_t i = 0; i < bms.cellCount && !rawFaultDetected; i++)
  {
    float v = bms.cells[i].voltage;
    if (isCellJump(i, v, cellMean[i], cellStdDev[i]))
    {
      rawFaultDetected = true;
      task2FaultSource = FAULT_SRC_ADC;
      task2FaultId = FAULT_ADC_JUMP;
      faultCellIndex = i;
      faultDetailA = v;
      faultDetailB = cellMean[i];
    }
  }

  // Priority Pass 4: WEAK CELL
  for (uint8_t i = 0; i < bms.cellCount && !rawFaultDetected; i++)
  {
    if (cellWeakLatched[i])
    {
      rawFaultDetected = true;
      task2FaultSource = FAULT_SRC_BATTERY;
      task2FaultId = FAULT_BATTERY_WEAK;
      faultCellIndex = i;
      faultDetailA = bms.cells[i].percent;
    }
  }

  // Priority Pass 5: HIGH VOLTAGE
  for (uint8_t i = 0; i < bms.cellCount && !rawFaultDetected; i++)
  {
    if (cellHighLatched[i])
    {
      rawFaultDetected = true;
      task2FaultSource = FAULT_SRC_BATTERY;
      task2FaultId = FAULT_BATTERY_HIGH;
      faultCellIndex = i;
      faultDetailA = bms.cells[i].percent;
    }
  }

  // --------------------------------------------------------
  // Build faultReason/faultReasonShort ONLY on an edge: the
  // winning fault's type or triggering cell differs from last
  // cycle's. While the same fault persists unchanged, the
  // existing strings are reused as-is - no allocation at all.
  // This is the actual fix for the heap-fragmentation hang.
  // --------------------------------------------------------
  if (rawFaultDetected)
  {
    if (task2FaultId != lastReportedFaultId || (int)faultCellIndex != lastReportedFaultCell)
    {
      String cellLabel = "Cell " + String(faultCellIndex + 1);
      String cellLabelShort = "C" + String(faultCellIndex + 1);

      switch (task2FaultId)
      {
        case FAULT_ADC_OUT_OF_RANGE:
          faultReason = cellLabel + " OUT_OF_RANGE (" + String(faultDetailA, 3) + "V)";
          faultReasonShort = cellLabelShort + " OUT-RANGE";
          break;
        case FAULT_ADC_FROZEN:
          faultReason = cellLabel + " FROZEN (pack active, this cell static)";
          faultReasonShort = cellLabelShort + " FROZEN";
          break;
        case FAULT_ADC_JUMP:
          faultReason = cellLabel + " JUMP (" + String(faultDetailA, 3) +
                         "V, window mean " + String(faultDetailB, 3) + "V)";
          faultReasonShort = cellLabelShort + " JUMP";
          break;
        case FAULT_BATTERY_WEAK:
          faultReason = cellLabel + " WEAK (" + String((int)faultDetailA) + "%)";
          faultReasonShort = cellLabelShort + " WEAK " + String((int)faultDetailA) + "%";
          break;
        case FAULT_BATTERY_HIGH:
          faultReason = cellLabel + " HIGH_VOLT (" + String((int)faultDetailA) + "%)";
          faultReasonShort = cellLabelShort + " HIGH " + String((int)faultDetailA) + "%";
          break;
        default:
          faultReason = cellLabel + " fault";
          faultReasonShort = cellLabelShort + " FAULT";
          break;
      }

      lastReportedFaultId = task2FaultId;
      lastReportedFaultCell = faultCellIndex;
    }
    // else: same fault, same cell as last cycle - strings unchanged, no allocation.
  }
  else if (lastReportedFaultId != FAULT_NONE)
  {
    // Edge: fault just cleared. Reset once, not every idle cycle.
    faultReason = "NONE";
    faultReasonShort = "NONE";
    lastReportedFaultId = FAULT_NONE;
    lastReportedFaultCell = -1;
  }

  // Push this cycle's readings into history AFTER all passes.
  for (uint8_t i = 0; i < bms.cellCount; i++)
  {
    pushCellHistory(i, bms.cells[i].voltage);
  }
}


// ============================================================
// FUNCTION: APPLY RELAY / LED / BUZZER OUTPUT (unchanged)
// ============================================================

void applyRelayOutputs()
{
  switch (relayState)
  {
    case RELAY_CONNECTED:
      digitalWrite(RELAY, LOW);
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER, LOW);
      break;

    case RELAY_TRIP_PENDING:
      digitalWrite(RELAY, LOW);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER, LOW);
      break;

    case RELAY_TRIPPED:
      digitalWrite(RELAY, HIGH);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BUZZER, HIGH);
      break;

    case RELAY_RECOVERING:
      digitalWrite(RELAY, HIGH);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER, LOW);
      break;
  }
}


// ============================================================
// TASK 2 - RELAY SAFETY STATE MACHINE (non-blocking, unchanged)
// ============================================================

void updateRelayStateMachine()
{
  unsigned long now = millis();

  switch (relayState)
  {
    case RELAY_CONNECTED:
    {
      if (rawFaultDetected)
      {
        debounceStartTime = now;
        logTransition(relayState, RELAY_TRIP_PENDING, faultReason);
        relayState = RELAY_TRIP_PENDING;

        lcdFaultLatched = true;
        Serial.println("[LCD LATCH] SET - fault page overriding display immediately.");
      }
      break;
    }

    case RELAY_TRIP_PENDING:
    {
      if (!rawFaultDetected)
      {
        logTransition(relayState, RELAY_CONNECTED, "condition cleared before debounce (filtered as noise)");
        relayState = RELAY_CONNECTED;

        lcdFaultLatched = false;
        Serial.println("[LCD LATCH] CLEARED - was filtered as noise, never confirmed as a trip.");
      }
      else if (now - debounceStartTime >= DEBOUNCE_MS)
      {
        logTransition(relayState, RELAY_TRIPPED, faultReason);
        relayState = RELAY_TRIPPED;
      }
      break;
    }

    case RELAY_TRIPPED:
    {
      if (!rawFaultDetected)
      {
        recoveryStartTime = now;
        logTransition(relayState, RELAY_RECOVERING, "fault cleared, starting timed recovery");
        relayState = RELAY_RECOVERING;
      }
      break;
    }

    case RELAY_RECOVERING:
    {
      if (rawFaultDetected)
      {
        logTransition(relayState, RELAY_TRIPPED, "fault reappeared during recovery: " + faultReason);
        relayState = RELAY_TRIPPED;
      }
      else if (now - recoveryStartTime >= RECOVERY_MS)
      {
        logTransition(relayState, RELAY_CONNECTED, "recovery period complete");
        relayState = RELAY_CONNECTED;

        lcdFaultLatched = false;
        Serial.println("[LCD LATCH] CLEARED - recovery complete, fault explicitly resolved.");
      }
      break;
    }
  }

  applyRelayOutputs();
}


// ============================================================
// TASK 4 - SUPERVISORY SYSTEM FAULT STATE MACHINE
//
// States: SYS_NORMAL, SYS_DEGRADED, SYS_FAILSAFE, SYS_SHUTDOWN
//
// Transition table:
//
//  CURRENT    | CONDITION                            | NEXT
//  -----------|---------------------------------------|----------
//  NORMAL     | no fault                              | NORMAL
//  NORMAL     | communication fault                   | DEGRADED
//  NORMAL     | battery / ADC / relay fault            | FAILSAFE
//  NORMAL     | critical (compound) condition          | SHUTDOWN
//  DEGRADED   | communication restored                 | NORMAL
//  DEGRADED   | serious fault (battery/ADC/relay)      | FAILSAFE
//  DEGRADED   | critical (compound) condition          | SHUTDOWN
//  FAILSAFE   | fault persists / verification fails    | FAILSAFE
//  FAILSAFE   | recovery verification succeeds         | NORMAL
//  FAILSAFE   | critical/repeated failure (timeout)    | SHUTDOWN
//  SHUTDOWN   | anything                               | SHUTDOWN
//
// There is no direct FAILSAFE -> NORMAL on the raw fault flag
// clearing - it must pass the full sequential recovery-verification
// ladder (runRecoveryVerification()) first.
//
// Fault isolation: activeFaultSource / activeFaultID are set once
// on entering FAILSAFE/DEGRADED/SHUTDOWN and held UNCHANGED for the
// entire duration the system is away from NORMAL - cleared only on
// a successful return to NORMAL.
// ============================================================

enum SystemState
{
  SYS_NORMAL,
  SYS_DEGRADED,
  SYS_FAILSAFE,
  SYS_SHUTDOWN
};

enum RecoveryCheck
{
  CHECK_ADC,
  CHECK_BATTERY,
  CHECK_RELAY,
  CHECK_COMMUNICATION,
  CHECK_COMPLETE
};

SystemState systemState = SYS_NORMAL;

FaultSource activeFaultSource = FAULT_SRC_NONE;
FaultID     activeFaultID     = FAULT_NONE;

unsigned long failsafeEnteredTime = 0;
RecoveryCheck recoveryStep = CHECK_ADC;
unsigned long recoveryCheckHoldStart = 0;

unsigned long commLossStart = 0;        // 0 = comms currently fine
unsigned long relayMismatchStart = 0;   // 0 = currently matching

constexpr unsigned long FAILSAFE_TO_SHUTDOWN_TIMEOUT_MS = 5000;
constexpr unsigned long COMM_LOSS_THRESHOLD_MS           = 10000;
constexpr unsigned long RELAY_MISMATCH_PERSIST_MS        = 500;
constexpr unsigned long RELAY_FB_SETTLE_MS               = 2000;   // ignore feedback right at boot
constexpr unsigned long RECOVERY_CHECK_HOLD_MS           = 1000;   // each ladder step must hold clean this long

const char* systemStateName(SystemState s)
{
  switch (s)
  {
    case SYS_NORMAL:   return "NORMAL";
    case SYS_DEGRADED: return "DEGRADED";
    case SYS_FAILSAFE: return "FAILSAFE";
    case SYS_SHUTDOWN: return "SHUTDOWN";
  }
  return "UNKNOWN";
}

String systemStateShort(SystemState s)
{
  switch (s)
  {
    case SYS_NORMAL:   return "NORM";
    case SYS_DEGRADED: return "DEGR";
    case SYS_FAILSAFE: return "FAIL";
    case SYS_SHUTDOWN: return "SHDN";
  }
  return "????";
}

const char* recoveryCheckName(RecoveryCheck c)
{
  switch (c)
  {
    case CHECK_ADC:           return "ADC";
    case CHECK_BATTERY:       return "BATTERY";
    case CHECK_RELAY:         return "RELAY";
    case CHECK_COMMUNICATION: return "COMMUNICATION";
    case CHECK_COMPLETE:      return "COMPLETE";
  }
  return "UNKNOWN";
}

// ------------------------------------------------------------
// FUNCTION: structured transition log.
// [FSM] t=<ms>ms | PREV -> NEXT | SRC=<source> | ID=<faultId> | <reason>
// ------------------------------------------------------------
void logSystemTransition(SystemState from, SystemState to, FaultSource src, FaultID id, const String& reason)
{
  Serial.print("[FSM] t=");
  Serial.print(millis());
  Serial.print("ms | ");
  Serial.print(systemStateName(from));
  Serial.print(" -> ");
  Serial.print(systemStateName(to));
  Serial.print(" | SRC=");
  Serial.print(faultSourceName(src));
  Serial.print(" | ID=");
  Serial.print(faultIdName(id));
  Serial.print(" | ");
  Serial.println(reason);
}

// ------------------------------------------------------------
// FUNCTION: mapSafetyFaultToTask4Fault()
// Thin accessor over the classification evaluateSafety() already
// performed inline (task2FaultSource/task2FaultId) - no re-deriving
// or string-parsing needed here.
// ------------------------------------------------------------
FaultID mapSafetyFaultToTask4Fault()
{
  return task2FaultId;
}

// ------------------------------------------------------------
// FUNCTION: detectRelayMismatch()
// Real hardware feedback via RELAY_FB (GPIO27 slide switch,
// simulating the relay's auxiliary contact). Requires the
// mismatch to PERSIST for RELAY_MISMATCH_PERSIST_MS before being
// reported (anti-chatter, same philosophy as Task 2), and is
// ignored entirely for the first RELAY_FB_SETTLE_MS after boot
// so the relay/feedback have time to settle before being judged.
// ------------------------------------------------------------
bool detectRelayMismatch()
{
  unsigned long now = millis();

  if (now < RELAY_FB_SETTLE_MS)
  {
    relayMismatchStart = 0;
    return false;
  }

  bool expectedHigh = (relayState == RELAY_TRIPPED || relayState == RELAY_RECOVERING);
  bool actualHigh   = (digitalRead(RELAY_FB) == HIGH);
  bool mismatchNow  = (expectedHigh != actualHigh);

  if (!mismatchNow)
  {
    relayMismatchStart = 0;
    return false;
  }

  if (relayMismatchStart == 0)
  {
    relayMismatchStart = now;
    return false;
  }

  return (now - relayMismatchStart) >= RELAY_MISMATCH_PERSIST_MS;
}

// ------------------------------------------------------------
// FUNCTION: evaluateCommunicationFault()
// Sustained (not momentary) WiFi/Blynk loss, using the same
// volatile flags Task 2's networkTask maintains on core 0.
// ------------------------------------------------------------
// ------------------------------------------------------------
// FUNCTION: evaluateCommunicationFault()
// Real communication detection only - sustained (>=10s, not
// momentary) WiFi/Blynk loss via Task 2's networkTask volatile
// flags. 4B/4C were verified with a temporary SW2/GPIO26 hardware
// injection fixture; that fixture has been fully removed here, so
// a communication fault now means an actual link drop, nothing else.
// ------------------------------------------------------------
bool evaluateCommunicationFault()
{
  unsigned long now = millis();

  bool commOk = g_wifiConnected && g_blynkConnected;

  if (commOk)
  {
    commLossStart = 0;
    return false;
  }

  if (commLossStart == 0)
  {
    commLossStart = now;
    return false;
  }

  return (now - commLossStart) >= COMM_LOSS_THRESHOLD_MS;
}

// ------------------------------------------------------------
// FUNCTION: forceRelaySafe()
// Explicit safety action taken on EVERY entry into SYS_SHUTDOWN,
// regardless of which path got there. Idempotent - a no-op if the
// relay is already tripped (the case for every currently-tested
// SHUTDOWN path), and closes the one gap where a pure relay
// mismatch could reach SHUTDOWN while Task 2's relay FSM still
// happened to be CONNECTED. Updates relayState itself (not just
// the GPIO) so LCD/Serial reporting and applyRelayOutputs()'s
// LED/buzzer indication stay consistent with the forced state.
// ------------------------------------------------------------
void forceRelaySafe()
{
  if (relayState != RELAY_TRIPPED)
  {
    Serial.println("[FSM] SHUTDOWN - forcing relay to safe (tripped) state.");
    relayState = RELAY_TRIPPED;
  }
  applyRelayOutputs();
}


// ------------------------------------------------------------
// FUNCTION: startFailsafeRecovery()
// Called once, exactly when the system enters FAILSAFE.
// ------------------------------------------------------------
void startFailsafeRecovery()
{
  failsafeEnteredTime = millis();
  recoveryStep = CHECK_ADC;
  recoveryCheckHoldStart = 0;
  Serial.println("[FSM] Entered FAILSAFE - recovery verification armed (ADC -> BATTERY -> RELAY -> COMMUNICATION).");
}

// ------------------------------------------------------------
// FUNCTION: runRecoveryVerification()
// Non-blocking sequential ladder. Each step must independently
// read clean and HOLD clean for RECOVERY_CHECK_HOLD_MS before
// advancing to the next. Any failure at any step - even on the
// very last check - resets the ladder back to CHECK_ADC, so a
// single bad reading can never produce a premature NORMAL.
// Does not even begin until Task 2's own relay FSM has fully
// returned to CONNECTED on its own (layered recovery timing).
// Returns true only on the exact cycle the full ladder completes.
// ------------------------------------------------------------
bool runRecoveryVerification(bool relayMismatchNow, bool batteryOrAdcFaultNow, bool commFaultNow)
{
  unsigned long now = millis();

  if (relayState != RELAY_CONNECTED)
  {
    recoveryStep = CHECK_ADC;
    recoveryCheckHoldStart = 0;
    return false;
  }

  bool currentPass;
  switch (recoveryStep)
  {
    case CHECK_ADC:
      currentPass = !(batteryOrAdcFaultNow && task2FaultSource == FAULT_SRC_ADC);
      break;
    case CHECK_BATTERY:
      currentPass = !(batteryOrAdcFaultNow && task2FaultSource == FAULT_SRC_BATTERY);
      break;
    case CHECK_RELAY:
      currentPass = !relayMismatchNow;
      break;
    case CHECK_COMMUNICATION:
      currentPass = !commFaultNow;
      break;
    default:
      currentPass = true;
      break;
  }

  if (!currentPass)
  {
    if (recoveryCheckHoldStart != 0)
    {
      Serial.print("[FSM] Recovery verification FAILED at step: ");
      Serial.println(recoveryCheckName(recoveryStep));
    }
    recoveryStep = CHECK_ADC;
    recoveryCheckHoldStart = 0;
    return false;
  }

  if (recoveryCheckHoldStart == 0)
  {
    recoveryCheckHoldStart = now;
    return false;
  }

  if (now - recoveryCheckHoldStart < RECOVERY_CHECK_HOLD_MS)
  {
    return false;
  }

  Serial.print("[FSM] Recovery check passed: ");
  Serial.println(recoveryCheckName(recoveryStep));

  if (recoveryStep == CHECK_COMMUNICATION)
  {
    recoveryStep = CHECK_COMPLETE;
    recoveryCheckHoldStart = 0;
    return true;
  }

  recoveryStep = (RecoveryCheck)(recoveryStep + 1);
  recoveryCheckHoldStart = 0;
  return false;
}

// ------------------------------------------------------------
// TASK 4 - MAIN SYSTEM FSM (non-blocking, call once per loop()
// after updateRelayStateMachine())
// ------------------------------------------------------------
// ------------------------------------------------------------
// FUNCTION: buildTransitionReasonText()
// Builds the human-readable reason text for a system-FSM
// transition log line. Deliberately NOT called every loop cycle -
// only from the 3 transition-logging sites inside
// updateSystemStateMachine(), which only execute on an actual
// state change. Same fix rationale as evaluateSafety()'s
// edge-triggered fault message: avoid rebuilding Strings at
// full loop frequency while a condition merely persists.
// ------------------------------------------------------------
String buildTransitionReasonText(bool relayMismatchNow, bool batteryOrAdcFaultNow, bool commFaultNow)
{
  if (relayMismatchNow)
  {
    return "Relay command/feedback mismatch (GPIO" + String(RELAY) + " vs GPIO" + String(RELAY_FB) + ")";
  }
  if (batteryOrAdcFaultNow)
  {
    return faultReason;
  }
  if (commFaultNow)
  {
    return "WiFi/Blynk link down >= " + String(COMM_LOSS_THRESHOLD_MS / 1000) + "s";
  }
  return "";
}

void updateSystemStateMachine()
{
  unsigned long now = millis();

  bool relayMismatch     = detectRelayMismatch();
  bool commFault         = evaluateCommunicationFault();
  bool batteryOrAdcFault = rawFaultDetected &&
                            (task2FaultSource == FAULT_SRC_BATTERY || task2FaultSource == FAULT_SRC_ADC);

  // Cheap (no heap allocation) - safe to compute every cycle.
  FaultSource detectedSource = FAULT_SRC_NONE;
  FaultID detectedId = FAULT_NONE;

  if (relayMismatch)
  {
    detectedSource = FAULT_SRC_RELAY;
    detectedId = FAULT_RELAY_MISMATCH;
  }
  else if (batteryOrAdcFault)
  {
    detectedSource = task2FaultSource;
    detectedId = mapSafetyFaultToTask4Fault();
  }
  else if (commFault)
  {
    detectedSource = FAULT_SRC_COMMUNICATION;
    detectedId = FAULT_COMMUNICATION;
  }

  // --------------------------------------------------------
  // Critical compound condition: relay mismatch occurring at the
  // same time as a confirmed battery/ADC fault means the very
  // mechanism meant to protect the pack can't be trusted while a
  // pack-level problem is also active. Escalates straight to
  // SHUTDOWN from ANY state, including directly from NORMAL.
  // --------------------------------------------------------
  bool compoundCritical = relayMismatch && batteryOrAdcFault;
  if (compoundCritical && systemState != SYS_SHUTDOWN)
  {
    activeFaultSource = FAULT_SRC_RELAY;
    activeFaultID = FAULT_RELAY_MISMATCH;
    logSystemTransition(systemState, SYS_SHUTDOWN, activeFaultSource, activeFaultID,
                         "critical compound fault: relay mismatch + battery/ADC fault simultaneously");
    systemState = SYS_SHUTDOWN;
    forceRelaySafe();
    return;
  }

  switch (systemState)
  {
    case SYS_NORMAL:
    {
      if (relayMismatch || batteryOrAdcFault)
      {
        activeFaultSource = detectedSource;
        activeFaultID = detectedId;
        logSystemTransition(SYS_NORMAL, SYS_FAILSAFE, activeFaultSource, activeFaultID,
                             buildTransitionReasonText(relayMismatch, batteryOrAdcFault, commFault));
        systemState = SYS_FAILSAFE;
        startFailsafeRecovery();
      }
      else if (commFault)
      {
        activeFaultSource = FAULT_SRC_COMMUNICATION;
        activeFaultID = FAULT_COMMUNICATION;
        logSystemTransition(SYS_NORMAL, SYS_DEGRADED, activeFaultSource, activeFaultID,
                             buildTransitionReasonText(relayMismatch, batteryOrAdcFault, commFault));
        systemState = SYS_DEGRADED;
      }
      break;
    }

    case SYS_DEGRADED:
    {
      if (relayMismatch || batteryOrAdcFault)
      {
        activeFaultSource = detectedSource;
        activeFaultID = detectedId;
        logSystemTransition(SYS_DEGRADED, SYS_FAILSAFE, activeFaultSource, activeFaultID,
                             buildTransitionReasonText(relayMismatch, batteryOrAdcFault, commFault));
        systemState = SYS_FAILSAFE;
        startFailsafeRecovery();
      }
      else if (!commFault)
      {
        logSystemTransition(SYS_DEGRADED, SYS_NORMAL, FAULT_SRC_NONE, FAULT_NONE, "communication restored");
        activeFaultSource = FAULT_SRC_NONE;
        activeFaultID = FAULT_NONE;
        systemState = SYS_NORMAL;
      }
      // else: stays DEGRADED
      break;
    }

    case SYS_FAILSAFE:
    {
      // A new confirmed fault may update activeFaultSource/activeFaultID
      // while the system remains in FAILSAFE. The system never leaves
      // FAILSAFE because of this update; recovery verification still
      // controls the only permitted transition back to NORMAL.
      if (detectedSource != FAULT_SRC_NONE &&
          (detectedSource != activeFaultSource || detectedId != activeFaultID))
      {
        activeFaultSource = detectedSource;
        activeFaultID = detectedId;

        Serial.print("[FSM] FAILSAFE fault updated -> SRC=");
        Serial.print(faultSourceName(activeFaultSource));
        Serial.print(" | ID=");
        Serial.println(faultIdName(activeFaultID));
      }

      bool timedOut = (now - failsafeEnteredTime) >= FAILSAFE_TO_SHUTDOWN_TIMEOUT_MS;
      if (timedOut)
      {
        logSystemTransition(SYS_FAILSAFE, SYS_SHUTDOWN, activeFaultSource, activeFaultID,
                             "FAILSAFE unresolved beyond " + String(FAILSAFE_TO_SHUTDOWN_TIMEOUT_MS / 1000) + "s");
        systemState = SYS_SHUTDOWN;
        forceRelaySafe();
        break;
      }

      bool recovered = runRecoveryVerification(relayMismatch, batteryOrAdcFault, commFault);
      if (recovered)
      {
        logSystemTransition(SYS_FAILSAFE, SYS_NORMAL, activeFaultSource, activeFaultID, "RECOVERY=PASS");
        activeFaultSource = FAULT_SRC_NONE;
        activeFaultID = FAULT_NONE;
        systemState = SYS_NORMAL;
      }
      break;
    }

    case SYS_SHUTDOWN:
    default:
      // Terminal, latched protective state. No automatic recovery -
      // this prototype requires an ESP32 reset to leave SHUTDOWN,
      // which is a deterministic, easy-to-demonstrate safety behavior.
      break;
  }
}


// ============================================================
// TASK 1 FUNCTIONS - write directly into bms (core 1 only, no mutex)
// ============================================================

float readVoltage(uint8_t pin)
{
  int rawValue = analogRead(pin);
  return (rawValue / 4095.0) * 3.3;
}

int voltageToPercent(float voltage)
{
  float percentage = (voltage / 3.3) * 100.0;
  percentage = constrain(percentage, 0.0, 100.0);
  return (int)percentage;
}

void runBatteryEngine()
{
  bms.cellCount = NUM_CELLS;

  for (uint8_t i = 0; i < NUM_CELLS; i++)
  {
    bms.cells[i].voltage = readVoltage(CELL_PINS[i]);   // live pot, nothing layered on top
    bms.cells[i].percent = voltageToPercent(bms.cells[i].voltage);
  }

  float total = 0.0;
  for (uint8_t i = 0; i < NUM_CELLS; i++) total += bms.cells[i].voltage;
  bms.averageVoltage = total / NUM_CELLS;
  bms.batterySoC = constrain((bms.averageVoltage / 3.3) * 100.0, 0.0, 100.0);

  bms.minimumVoltage = bms.cells[0].voltage;
  bms.maximumVoltage = bms.cells[0].voltage;
  bms.weakestCell = 0;
  bms.strongestCell = 0;

  for (uint8_t i = 1; i < NUM_CELLS; i++)
  {
    if (bms.cells[i].voltage < bms.minimumVoltage)
    {
      bms.minimumVoltage = bms.cells[i].voltage;
      bms.weakestCell = i;
    }
    if (bms.cells[i].voltage > bms.maximumVoltage)
    {
      bms.maximumVoltage = bms.cells[i].voltage;
      bms.strongestCell = i;
    }
  }
  bms.voltageImbalance = bms.maximumVoltage - bms.minimumVoltage;

  if (!previousDataAvailable)
  {
    previousImbalance = bms.voltageImbalance;
    previousDataAvailable = true;
    strcpy(bms.imbalanceTrend, "START");
  }
  else
  {
    float difference = bms.voltageImbalance - previousImbalance;
    if (difference > TREND_TOLERANCE)       strcpy(bms.imbalanceTrend, "RISING");
    else if (difference < -TREND_TOLERANCE) strcpy(bms.imbalanceTrend, "FALLING");
    else                                     strcpy(bms.imbalanceTrend, "STABLE");
    previousImbalance = bms.voltageImbalance;
  }

  if (bms.batterySoC >= 80.0)      bms.adaptiveThreshold = 0.05;
  else if (bms.batterySoC >= 50.0) bms.adaptiveThreshold = 0.08;
  else if (bms.batterySoC >= 20.0) bms.adaptiveThreshold = 0.10;
  else                              bms.adaptiveThreshold = 0.12;

  bool weakCellDetected = false;
  bool highVoltageDetected = false;
  for (uint8_t i = 0; i < NUM_CELLS; i++)
  {
    if (bms.cells[i].percent < 30) weakCellDetected = true;
    if (bms.cells[i].percent > 90) highVoltageDetected = true;
  }
  bool imbalanceDetected = bms.voltageImbalance > bms.adaptiveThreshold;

  if (weakCellDetected)        strcpy(bms.status, "WEAK CELL");
  else if (highVoltageDetected) strcpy(bms.status, "HIGH VOLT");
  else if (imbalanceDetected)   strcpy(bms.status, "IMBALANCE");
  else                          strcpy(bms.status, "NORMAL");

  // Publish for core 0 - the ONLY place bmsShared is touched from core 1.
  publishSharedSnapshot();
}


// ============================================================
// FUNCTION: getSnapshot() - CORE 1 USE ONLY.
// ============================================================

const BMSSnapshot& getSnapshot()
{
  return bms;
}


// ============================================================
// FUNCTION: SERIAL PRINT (core 1)
// ============================================================

void printBatteryData()
{
  const BMSSnapshot& s = getSnapshot();

  Serial.println();
  Serial.println("================================================");
  Serial.println("  MODULAR BMS - TASK 1+2+3+4 (fault FSM)");
  Serial.println("================================================");

  Serial.print("Number of Cells : "); Serial.println(s.cellCount);
  Serial.println();

  for (uint8_t i = 0; i < s.cellCount; i++)
  {
    Serial.print("Cell "); Serial.print(i + 1); Serial.print(" : ");
    Serial.print(s.cells[i].voltage, 3); Serial.print(" V | ");
    Serial.print(s.cells[i].percent); Serial.println(" %");
  }

  Serial.println("-----------------------------------------------");
  Serial.print("Average Voltage : "); Serial.print(s.averageVoltage, 3); Serial.println(" V");
  Serial.print("Battery SoC     : "); Serial.print(s.batterySoC, 1); Serial.println(" %");
  Serial.print("Voltage Imbal.  : "); Serial.print(s.voltageImbalance, 3); Serial.println(" V");
  Serial.print("Imbalance Trend : "); Serial.println(s.imbalanceTrend);
  Serial.print("Adaptive Limit  : "); Serial.print(s.adaptiveThreshold, 3); Serial.println(" V");
  Serial.print("Battery Status  : "); Serial.println(s.status);
  Serial.println("-----------------------------------------------");
  Serial.print("Relay State     : "); Serial.println(relayStateName(relayState));
  Serial.print("Fault Reason    : "); Serial.println(faultReason);
  Serial.print("Relay FB Pin    : "); Serial.println(digitalRead(RELAY_FB) == HIGH ? "HIGH" : "LOW");
  Serial.print("Wi-Fi Status    : "); Serial.println(g_wifiConnected ? "CONNECTED" : "DISCONNECTED");
  Serial.print("Blynk HTTP      : "); Serial.println(g_blynkConnected ? "CONNECTED" : "DISCONNECTED");
  Serial.println("-----------------------------------------------");
  Serial.print("System State    : "); Serial.println(systemStateName(systemState));
  Serial.print("Fault Source    : "); Serial.println(faultSourceName(activeFaultSource));
  Serial.print("Fault ID        : "); Serial.println(faultIdName(activeFaultID));
  if (systemState == SYS_FAILSAFE)
  {
    Serial.print("Recovery Step   : "); Serial.println(recoveryCheckName(recoveryStep));
  }
  Serial.println("================================================");
}


// ============================================================
// TASK 3 - FLICKER-FREE LCD ENGINE
//
// Page selection is now driven by Task 4's systemState (section 18
// of the spec): NORMAL/DEGRADED keep rotating through the 3 normal
// pages (DEGRADED adds a short indicator on the System page, not a
// full override); FAILSAFE and SHUTDOWN each get their own
// dedicated override page. Rotation only advances in NORMAL/DEGRADED,
// so a fault never gets "skipped past" and resumes from the same
// point once systemState returns to NORMAL.
//
// Refresh interval justification (unchanged from Task 3):
// - CONTENT_REFRESH_INTERVAL_MS = 250ms (4Hz)
// - PAGE_ROTATE_INTERVAL_MS = 3000ms
// ============================================================

enum LcdPage
{
  PAGE_BATTERY,
  PAGE_SYSTEM,
  PAGE_TELEMETRY,
  PAGE_COUNT,       // number of ROTATING pages (3) - not a real page
  PAGE_FAULT,       // virtual page - SYS_FAILSAFE override
  PAGE_SHUTDOWN     // virtual page - SYS_SHUTDOWN override
};

uint8_t currentPageIndex = 0;

constexpr unsigned long PAGE_ROTATE_INTERVAL_MS      = 3000;
constexpr unsigned long CONTENT_REFRESH_INTERVAL_MS  = 250;

unsigned long lastPageRotate = 0;

char lcdOnScreen[2][17] = { "                ", "                " };

String relayStateShort(RelayState s)
{
  switch (s)
  {
    case RELAY_CONNECTED:    return "CONN";
    case RELAY_TRIP_PENDING: return "PEND";
    case RELAY_TRIPPED:      return "TRIP";
    case RELAY_RECOVERING:   return "RECV";
  }
  return "????";
}

String padTo16(String s)
{
  if (s.length() > 16) s = s.substring(0, 16);
  while (s.length() < 16) s += ' ';
  return s;
}

void buildPage(uint8_t page, String& line0, String& line1)
{
  const BMSSnapshot& s = getSnapshot();

  switch (page)
  {
    case PAGE_BATTERY:
      line0 = padTo16(String("Batt:") + s.status);
      line1 = padTo16("SoC:" + String((int)s.batterySoC) + "% Th:" + String(s.adaptiveThreshold, 2) + "V");
      break;

    case PAGE_SYSTEM:
      // DEGRADED shows as a short indicator here rather than a full override.
      line0 = padTo16("Relay:" + relayStateShort(relayState) + " " + systemStateShort(systemState));
      line1 = padTo16("W:C" + String(s.weakestCell + 1) + " S:C" + String(s.strongestCell + 1) +
                       " " + (g_blynkConnected ? "BLK-OK" : "BLK-NO"));
      break;

    case PAGE_TELEMETRY:
      line0 = padTo16("Avg:" + String(s.averageVoltage, 2) + "V Im:" + String(s.voltageImbalance, 2));
      line1 = padTo16(String("Trend:") + s.imbalanceTrend + " " + (g_wifiConnected ? "WF-OK" : "WF-NO"));
      break;

    case PAGE_FAULT:
      line0 = padTo16("!! FAILSAFE !!");
      line1 = padTo16(faultSourceName(activeFaultSource) + String(" ") + faultIdShort(activeFaultID));
      break;

    case PAGE_SHUTDOWN:
    default:
      line0 = padTo16("!! SHUTDOWN !!");
      line1 = padTo16(faultIdShort(activeFaultID));
      break;
  }
}

void renderDiff(const String& line0, const String& line1)
{
  const String* lines[2] = { &line0, &line1 };

  for (uint8_t row = 0; row < 2; row++)
  {
    for (uint8_t col = 0; col < 16; col++)
    {
      char c = (*lines[row]).charAt(col);
      if (c != lcdOnScreen[row][col])
      {
        lcd.setCursor(col, row);
        lcd.print(c);
        lcdOnScreen[row][col] = c;
      }
    }
  }
}

void updateLCD()
{
  unsigned long now = millis();

  bool overrideActive = (systemState == SYS_FAILSAFE || systemState == SYS_SHUTDOWN);

  // Page rotation only advances in NORMAL/DEGRADED, so a fault never
  // gets "skipped past" - rotation resumes from the same point once
  // systemState returns to NORMAL.
  if (!overrideActive && (now - lastPageRotate >= PAGE_ROTATE_INTERVAL_MS))
  {
    lastPageRotate = now;
    currentPageIndex = (currentPageIndex + 1) % PAGE_COUNT;
  }

  uint8_t pageToShow;
  if (systemState == SYS_SHUTDOWN)      pageToShow = PAGE_SHUTDOWN;
  else if (systemState == SYS_FAILSAFE) pageToShow = PAGE_FAULT;
  else                                   pageToShow = currentPageIndex;

  String line0, line1;
  buildPage(pageToShow, line0, line1);

  renderDiff(line0, line1);
}


// ============================================================
// CORE 0 TASK - Wi-Fi and Blynk HTTP API telemetry.
// Reads bmsShared only through copySharedSnapshot(); the realtime
// Blynk library/socket is not used.
// ============================================================






// ============================================================
// TASKS 5 + 6 - EVENT-DRIVEN HTTPS TELEMETRY + ANALYTICS
// Network work runs only on Core 0. Safety/FSM logic remains on Core 1.
// Queue is RAM-only and survives Wi-Fi outages, not ESP32 resets.
// ============================================================

constexpr uint8_t EVENT_CAPACITY = 24;
constexpr uint32_t WIFI_RETRY_MS = 5000;
constexpr uint32_t SEND_RETRY_MS = 3000;
constexpr uint32_t HEARTBEAT_MS = 30000;
constexpr uint32_t MIN_SEND_GAP_MS = 500;
constexpr float CELL_EVENT_DELTA = 0.05f;
constexpr float SOC_EVENT_DELTA = 3.0f;
constexpr float IMBALANCE_EVENT_DELTA = 0.03f;

struct Task56Snapshot {
  BMSSnapshot battery;
  uint8_t system;
  uint8_t relay;
  uint8_t fault;
  uint8_t faultSource;
  uint32_t capturedMs;
};

Task56Snapshot task56Shared;
bool task56SharedReady = false;

enum NetworkManagerState : uint8_t {
  NET_CONNECTING = 0,
  NET_CONNECTED,
  NET_RETRY_WAIT
};

NetworkManagerState networkManagerState = NET_CONNECTING;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

constexpr uint8_t FAULT_HISTORY_CAPACITY = 16;
constexpr uint32_t FAULT_FREQUENCY_WINDOW_MS = 600000UL; // rolling 10-minute window
uint32_t faultOccurrenceTimes[FAULT_HISTORY_CAPACITY] = {0};
uint8_t faultOccurrenceHead = 0;
uint8_t faultOccurrenceCount = 0;

struct TelemetryEvent {
  Task56Snapshot data;
  uint32_t sequence;
  uint32_t eventMs;
  uint32_t faultCountAtCapture;
  uint8_t recentFaultsAtCapture;
  uint8_t previousSystem;
  uint8_t previousRelay;
  uint8_t previousFault;
  uint8_t previousFaultSource;
  uint8_t networkState;
  int16_t rssi;
  bool wifiConnected;
  bool timelineLogged;
  uint8_t reason; // 1 boot, 2 FSM/relay transition, 3 battery change, 4 heartbeat, 5 Wi-Fi link
};

TelemetryEvent eventQueue[EVENT_CAPACITY];
uint8_t queueHead = 0;
uint8_t queueCount = 0;
uint32_t nextEventSequence = 1;
uint32_t droppedEventCount = 0;
uint32_t sentEventCount = 0;
uint32_t totalFaultCount = 0;
uint32_t lastOnlineStart = 0;
uint32_t accumulatedOnlineMs = 0;
uint32_t lastSuccessfulHttpMs = 0;
uint32_t lastWifiAttemptMs = 0;
bool previousWifi = false;
bool previousSnapshotValid = false;
Task56Snapshot previousSnapshot;

void recordFaultOccurrence(uint32_t now)
{
  faultOccurrenceTimes[faultOccurrenceHead] = now;
  faultOccurrenceHead = (faultOccurrenceHead + 1) % FAULT_HISTORY_CAPACITY;
  if (faultOccurrenceCount < FAULT_HISTORY_CAPACITY) faultOccurrenceCount++;
}

uint8_t countRecentFaultOccurrences(uint32_t now)
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < faultOccurrenceCount; ++i)
  {
    uint32_t timestamp = faultOccurrenceTimes[i];
    if ((uint32_t)(now - timestamp) <= FAULT_FREQUENCY_WINDOW_MS) count++;
  }
  return count;
}

const char* networkManagerStateName(NetworkManagerState state)
{
  switch (state)
  {
    case NET_CONNECTING:  return "CONNECTING";
    case NET_CONNECTED:   return "CONNECTED";
    case NET_RETRY_WAIT:  return "RETRY_WAIT";
  }
  return "UNKNOWN";
}

const char* rssiQualityName(int rssi)
{
  if (rssi <= -100) return "NO_LINK";
  if (rssi >= -50) return "EXCELLENT";
  if (rssi >= -60) return "GOOD";
  if (rssi >= -70) return "FAIR";
  if (rssi >= -80) return "WEAK";
  return "VERY_WEAK";
}

String urlEncodeQueryValue(const char* input)
{
  String encoded;
  if (!input) return encoded;
  const char hexDigits[] = "0123456789ABCDEF";
  while (*input) {
    uint8_t c = (uint8_t)*input++;
    bool safe = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') || c == '-' || c == '_' ||
                c == '.' || c == '~';
    if (safe) encoded += (char)c;
    else {
      encoded += '%';
      encoded += hexDigits[(c >> 4) & 0x0F];
      encoded += hexDigits[c & 0x0F];
    }
  }
  return encoded;
}

bool copyTask56Snapshot(Task56Snapshot& out)
{
  if (xSemaphoreTake(bmsMutex, 0) != pdTRUE) return false;
  out = task56Shared;
  bool ready = task56SharedReady;
  xSemaphoreGive(bmsMutex);
  return ready;
}

bool sameState(const Task56Snapshot& a, const Task56Snapshot& b)
{
  return a.system == b.system &&
         a.relay == b.relay &&
         a.fault == b.fault &&
         a.faultSource == b.faultSource;
}

bool significantBatteryChange(const Task56Snapshot& a, const Task56Snapshot& b)
{
  if (fabsf(a.battery.batterySoC - b.battery.batterySoC) >= SOC_EVENT_DELTA) return true;
  if (fabsf(a.battery.voltageImbalance - b.battery.voltageImbalance) >= IMBALANCE_EVENT_DELTA) return true;
  for (uint8_t i = 0; i < NUM_CELLS; ++i)
  {
    if (fabsf(a.battery.cells[i].voltage - b.battery.cells[i].voltage) >= CELL_EVENT_DELTA) return true;
  }
  return false;
}

const char* eventReasonName(uint8_t reason)
{
  switch (reason) {
    case 1: return "BOOT";
    case 2: return "FSM_TRANSITION";
    case 3: return "BATTERY_CHANGE";
    case 4: return "HEARTBEAT";
    case 5: return "WIFI_LINK_CHANGE";
  }
  return "EVENT";
}

String buildTransitionText(const TelemetryEvent& event)
{
  if (event.reason == 2)
  {
    String text = "SYS:";
    text += systemStateName((SystemState)event.previousSystem);
    text += ">";
    text += systemStateName((SystemState)event.data.system);
    text += "; RELAY:";
    text += relayStateName((RelayState)event.previousRelay);
    text += ">";
    text += relayStateName((RelayState)event.data.relay);
    text += "; FAULT:";
    text += faultIdName((FaultID)event.previousFault);
    text += ">";
    text += faultIdName((FaultID)event.data.fault);
    text += "; SRC:";
    text += faultSourceName((FaultSource)event.data.faultSource);
    text += "; SEQ:";
    text += String(event.sequence);
    return text;
  }

  if (event.reason == 5)
  {
    String text = event.wifiConnected ? "WIFI_LINK_RESTORED" : "WIFI_LINK_LOST";
    text += "; NETWORK_STATE:";
    text += networkManagerStateName((NetworkManagerState)event.networkState);
    text += "; SEQ:";
    text += String(event.sequence);
    return text;
  }

  return String(eventReasonName(event.reason)) + " #" + String(event.sequence);
}

bool enqueueEvent(const Task56Snapshot& snapshot, uint8_t reason)
{
  if (queueCount >= EVENT_CAPACITY)
  {
    // Fixed-size FIFO overflow policy: discard oldest, preserve newest state.
    queueHead = (queueHead + 1) % EVENT_CAPACITY;
    queueCount--;
    droppedEventCount++;
    Serial.println("[TASK5] Queue full: oldest event discarded.");
  }

  uint8_t tail = (queueHead + queueCount) % EVENT_CAPACITY;
  TelemetryEvent& event = eventQueue[tail];
  event.data = snapshot;
  event.sequence = nextEventSequence++;
  event.eventMs = millis();
  event.faultCountAtCapture = totalFaultCount;
  event.recentFaultsAtCapture = countRecentFaultOccurrences(event.eventMs);
  event.previousSystem = previousSnapshotValid ? previousSnapshot.system : snapshot.system;
  event.previousRelay = previousSnapshotValid ? previousSnapshot.relay : snapshot.relay;
  event.previousFault = previousSnapshotValid ? previousSnapshot.fault : snapshot.fault;
  event.previousFaultSource = previousSnapshotValid ? previousSnapshot.faultSource : snapshot.faultSource;
  event.networkState = (uint8_t)networkManagerState;
  event.wifiConnected = g_wifiConnected;
  event.rssi = g_wifiConnected ? (int16_t)WiFi.RSSI() : (int16_t)-127;
  event.timelineLogged = false;
  event.reason = reason;
  queueCount++;

  Serial.printf("[TASK5] Queued event #%lu (%s), queue=%u\n",
                (unsigned long)event.sequence,
                eventReasonName(reason),
                (unsigned int)queueCount);
  return true;
}

float calculateRiskScore(const TelemetryEvent& event)
{
  // Project risk indicator (heuristic, not a validated SOH prediction):
  // imbalance 30 + SoC 15 + trend 15 + recent fault frequency 15 + FSM state 25.
  const Task56Snapshot& s = event.data;

  float imbalanceLimit = (s.battery.adaptiveThreshold > 0.01f)
                           ? s.battery.adaptiveThreshold : 0.20f;
  float imbalanceRisk = constrain(
      (s.battery.voltageImbalance / imbalanceLimit) * 30.0f, 0.0f, 30.0f);
  float socRisk = constrain((100.0f - s.battery.batterySoC) * 0.15f, 0.0f, 15.0f);

  float trendRisk = 0.0f;
  if (strcmp(s.battery.imbalanceTrend, "RISING") == 0) trendRisk = 15.0f;
  else if (strcmp(s.battery.imbalanceTrend, "START") == 0) trendRisk = 5.0f;

  float faultFrequencyRisk = constrain(
      (float)event.recentFaultsAtCapture * 5.0f, 0.0f, 15.0f);

  float stateRisk = (s.system == SYS_SHUTDOWN) ? 25.0f :
                    (s.system == SYS_FAILSAFE) ? 20.0f :
                    (s.system == SYS_DEGRADED) ? 8.0f : 0.0f;

  return constrain(imbalanceRisk + socRisk + trendRisk +
                   faultFrequencyRisk + stateRisk, 0.0f, 100.0f);
}

String maintenanceAdvice(const Task56Snapshot& s, float risk, uint8_t recentFaults)
{
  if (s.system == SYS_SHUTDOWN) return "SHUTDOWN: inspect fault before reset";
  if (s.system == SYS_FAILSAFE) return "URGENT: isolate pack and inspect fault";
  if (s.fault == FAULT_COMMUNICATION) return "Check Wi-Fi and Blynk connectivity";
  if (recentFaults >= 3) return "Repeated faults: inspect pack and protection hardware";
  if (strcmp(s.battery.imbalanceTrend, "RISING") == 0 &&
      s.battery.voltageImbalance > s.battery.adaptiveThreshold)
    return "Rising imbalance: inspect cell balance";
  if (s.battery.voltageImbalance > s.battery.adaptiveThreshold)
    return "Inspect cell imbalance; check balancing";
  if (s.battery.batterySoC < 20.0f) return "Low SoC: recharge battery pack";
  if (risk >= 60.0f) return "High risk: inspect battery condition";
  if (risk >= 35.0f) return "Monitor battery trend and cell voltages";
  return "Battery operating normally";
}

bool logBlynkTransitionEvent(const TelemetryEvent& event)
{
  if (event.reason != 2) return true;

  // Create this custom Event in the Blynk Template with code:
  // bms_transition. Enable Timeline recording for the event.
  String description = buildTransitionText(event);
  if (description.length() > 240) description.remove(240);

  WiFiClientSecure client;
  client.setInsecure(); // Prototype only; validate certificates in production.
  client.setTimeout(4000);

  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(4000);
  http.setReuse(false);

  String url = "https://blynk.cloud/external/api/logEvent?token=";
  url += BLYNK_AUTH_TOKEN;
  url += "&code=bms_transition&description=";
  url += urlEncodeQueryValue(description.c_str());

  if (!http.begin(client, url))
  {
    Serial.println("[TASK6] Could not start Blynk transition-event request.");
    return false;
  }

  int code = http.GET();
  http.end();

  if (code >= 200 && code < 300)
  {
    Serial.printf("[TASK6] Blynk transition logged: %s\n", description.c_str());
    return true;
  }

  Serial.printf("[TASK6] Transition timeline logging failed (HTTP %d). Check bms_transition Event setup.\n", code);
  return false;
}

bool sendBlynkEvent(const TelemetryEvent& event, bool replayed,
                    uint8_t queueDepthAfterSuccess)
{
  const BMSSnapshot& s = event.data.battery;
  float risk = calculateRiskScore(event);
  String advice = maintenanceAdvice(event.data, risk, event.recentFaultsAtCapture);
  String transitionText = buildTransitionText(event);
  uint32_t now = millis();
  uint32_t eventAgeSec = (uint32_t)(now - event.eventMs) / 1000UL;
  uint32_t uptimeAtCaptureSec = event.eventMs / 1000UL;

  WiFiClientSecure client;
  client.setInsecure(); // Prototype only; use certificate validation in production.
  client.setTimeout(4000);

  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(4000);
  http.setReuse(false);

  String url;
  url.reserve(2600);
  url = "https://blynk.cloud/external/api/batch/update?token=";
  url += BLYNK_AUTH_TOKEN;

  // Existing Task 1-4 datastreams V0-V13.
  url += "&V0=" + String(s.averageVoltage, 3);
  for (uint8_t i = 0; i < NUM_CELLS; ++i)
    url += "&V" + String(i + 1) + "=" + String(s.cells[i].voltage, 3);
  url += "&V5=" + urlEncodeQueryValue(("Cell " + String(s.weakestCell + 1)).c_str());
  url += "&V6=" + urlEncodeQueryValue(("Cell " + String(s.strongestCell + 1)).c_str());
  url += "&V7=" + String(s.voltageImbalance, 3);
  url += "&V8=" + urlEncodeQueryValue(s.status);
  url += "&V9=" + String(s.batterySoC, 1);
  url += "&V10=" + String(s.minimumVoltage, 3);
  url += "&V11=" + String(s.maximumVoltage, 3);
  url += "&V12=" + String(s.adaptiveThreshold, 3);
  url += "&V13=" + urlEncodeQueryValue(s.imbalanceTrend);

  // Task 5/6 datastreams V14-V36.
  url += "&V14=" + String(event.data.system);
  url += "&V15=" + String(event.data.relay);
  url += "&V16=" + String(event.data.fault);
  url += "&V17=" + String(event.rssi);
  url += "&V18=" + String(queueDepthAfterSuccess);
  url += "&V19=" + String((int)roundf(risk));
  url += "&V20=" + urlEncodeQueryValue(advice.c_str());
  url += "&V21=" + String(event.faultCountAtCapture);
  url += "&V22=" + String(droppedEventCount);
  url += "&V23=" + String(uptimeAtCaptureSec);
  url += "&V24=" + String(sentEventCount + 1);
  url += "&V25=" + urlEncodeQueryValue(systemStateName((SystemState)event.data.system));
  url += "&V26=" + urlEncodeQueryValue(relayStateShort((RelayState)event.data.relay).c_str());
  String eventText = String(eventReasonName(event.reason)) + " #" + String(event.sequence);
  url += "&V27=" + urlEncodeQueryValue(eventText.c_str());
  url += "&V28=" + urlEncodeQueryValue(replayed ? "REPLAYED" : "LIVE");
  url += "&V29=" + String(eventAgeSec);
  url += "&V30=" + String(event.sequence);
  url += "&V31=" + urlEncodeQueryValue(faultSourceName((FaultSource)event.data.faultSource));
  url += "&V32=" + urlEncodeQueryValue(faultIdName((FaultID)event.data.fault));
  url += "&V33=" + urlEncodeQueryValue(transitionText.c_str());
  url += "&V34=" + urlEncodeQueryValue(networkManagerStateName((NetworkManagerState)event.networkState));
  url += "&V35=" + urlEncodeQueryValue(rssiQualityName(event.rssi));
  url += "&V36=" + String(event.recentFaultsAtCapture);

  if (!http.begin(client, url)) return false;
  int code = http.GET();
  http.end();

  if (code < 200 || code >= 300) return false;

  return true;
}

// ============================================================
// SETUP (no blocking calls, no delay())
// ============================================================

void networkTask(void* param)
{
  (void)param;

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.begin(ssid, pass);

  networkManagerState = NET_CONNECTING;
  lastWifiAttemptMs = millis();

  uint32_t lastSendAttempt = 0;
  uint32_t lastHeartbeat = 0;

  for (;;)
  {
    uint32_t now = millis();
    bool physicalLinkUp = (WiFi.status() == WL_CONNECTED);

    // Explicit, non-blocking Wi-Fi connection state machine.
    switch (networkManagerState)
    {
      case NET_CONNECTING:
        if (physicalLinkUp)
        {
          networkManagerState = NET_CONNECTED;
          Serial.println("[TASK5] Wi-Fi state: CONNECTING -> CONNECTED");
        }
        else if ((uint32_t)(now - lastWifiAttemptMs) >= WIFI_CONNECT_TIMEOUT_MS)
        {
          networkManagerState = NET_RETRY_WAIT;
          lastWifiAttemptMs = now;
          Serial.println("[TASK5] Wi-Fi connection timed out; entering RETRY_WAIT.");
        }
        break;

      case NET_CONNECTED:
        if (!physicalLinkUp)
        {
          networkManagerState = NET_RETRY_WAIT;
          lastWifiAttemptMs = now;
          Serial.println("[TASK5] Wi-Fi link lost; entering RETRY_WAIT.");
        }
        break;

      case NET_RETRY_WAIT:
        if (physicalLinkUp)
        {
          networkManagerState = NET_CONNECTED;
          Serial.println("[TASK5] Wi-Fi state: RETRY_WAIT -> CONNECTED");
        }
        else if ((uint32_t)(now - lastWifiAttemptMs) >= WIFI_RETRY_MS)
        {
          lastWifiAttemptMs = now;
          WiFi.begin(ssid, pass);
          networkManagerState = NET_CONNECTING;
          Serial.println("[TASK5] Wi-Fi retry initiated (non-blocking).");
        }
        break;
    }

    bool wifi = (networkManagerState == NET_CONNECTED) &&
                (WiFi.status() == WL_CONNECTED);
    g_wifiConnected = wifi;

    if (!wifi) g_blynkConnected = false;

    // Capture both link-down and link-restored events. These are queued
    // before any later battery/FSM events, preserving event order.
    if (wifi && !previousWifi)
    {
      if (previousSnapshotValid) enqueueEvent(previousSnapshot, 5);
      lastOnlineStart = now;
    }
    else if (!wifi && previousWifi)
    {
      if (lastOnlineStart != 0)
      {
        accumulatedOnlineMs += now - lastOnlineStart;
        lastOnlineStart = 0;
      }
      if (previousSnapshotValid) enqueueEvent(previousSnapshot, 5);
    }
    previousWifi = wifi;

    // --------------------------------------------------------
    // EVENT DETECTION
    // --------------------------------------------------------
    Task56Snapshot current = previousSnapshot;
    if (copyTask56Snapshot(current))
    {
      if (!previousSnapshotValid)
      {
        previousSnapshot = current;
        previousSnapshotValid = true;
        enqueueEvent(current, 1);
        lastHeartbeat = now;
      }
      else
      {
        bool fsmChanged = !sameState(current, previousSnapshot);
        bool batteryChanged = significantBatteryChange(current, previousSnapshot);

        if (fsmChanged)
        {
          // Count a new fault occurrence only when a new non-NONE fault
          // appears or its source/ID changes. FAILSAFE -> SHUTDOWN for
          // the same fault is an escalation, not a second occurrence.
          bool newFaultOccurrence =
              (current.fault != FAULT_NONE) &&
              ((current.fault != previousSnapshot.fault) ||
               (current.faultSource != previousSnapshot.faultSource));

          if (newFaultOccurrence)
          {
            totalFaultCount++;
            recordFaultOccurrence(now);
          }

          enqueueEvent(current, 2);
          previousSnapshot = current;
        }
        else if (batteryChanged)
        {
          enqueueEvent(current, 3);
          previousSnapshot = current;
        }
        else if ((uint32_t)(now - lastHeartbeat) >= HEARTBEAT_MS)
        {
          // A low-rate heartbeat keeps uptime and historical charts alive;
          // all other transmissions remain event/threshold driven.
          enqueueEvent(current, 4);
          lastHeartbeat = now;
        }
      }
    }

    // --------------------------------------------------------
    // FIFO QUEUE TRANSMISSION
    // --------------------------------------------------------
    uint32_t sendGap = g_blynkConnected ? MIN_SEND_GAP_MS : SEND_RETRY_MS;
    if (wifi && queueCount > 0 &&
        (uint32_t)(now - lastSendAttempt) >= sendGap)
    {
      lastSendAttempt = now;
      TelemetryEvent& event = eventQueue[queueHead];
      bool replayed = (uint32_t)(now - event.eventMs) > 2000UL;
      uint8_t queueDepthAfterSuccess = (queueCount > 0) ? (queueCount - 1) : 0;

      // Log backend state transitions once. If datastream transmission
      // subsequently fails, timelineLogged prevents duplicate timeline
      // records when the same FIFO item is retried.
      bool timelineOk = true;
      if (event.reason == 2 && !event.timelineLogged)
      {
        timelineOk = logBlynkTransitionEvent(event);
        if (timelineOk) event.timelineLogged = true;
      }

      bool ok = timelineOk && sendBlynkEvent(event, replayed, queueDepthAfterSuccess);
      g_blynkConnected = ok;

      if (ok)
      {
        queueHead = (queueHead + 1) % EVENT_CAPACITY;
        queueCount--;
        sentEventCount++;
        lastSuccessfulHttpMs = now;

        Serial.printf("[TASK5] Sent event #%lu (%s), queue=%u\n",
                      (unsigned long)event.sequence,
                      replayed ? "REPLAYED" : "LIVE",
                      (unsigned int)queueCount);
      }
      else
      {
        Serial.println("[TASK5] Blynk HTTP update failed; event retained for retry.");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ============================================================
// SETUP (no blocking calls, no delay())
// ============================================================

void setup()
{
  Serial.begin(115200);
  disableCore0WDT();
  disableCore1WDT();

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(RELAY_FB, INPUT);   // TASK 4 - relay feedback (slide switch, SW1)

  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(RELAY, LOW);

  // Single one-time init clear. No splash text printed, so the
  // diff engine's "all spaces" starting buffer matches the screen.
  lcd.init();
  lcd.backlight();
  lcd.clear();

  bmsMutex = xSemaphoreCreateMutex();

#if ENABLE_NETWORKING
  xTaskCreatePinnedToCore(
    networkTask,
    "NetworkTask",
    8192,
    NULL,
    1,
    NULL,
    0
  );
#else
  Serial.println("[DIAGNOSTIC] Networking DISABLED (ENABLE_NETWORKING=0) - no WiFi/Blynk/core-0 task.");
#endif

  runBatteryEngine();
  applyRelayOutputs();

  Serial.println();
  Serial.println("================================================");
  Serial.println("   COMMON BASELINE STARTED (Tasks 1-4, SW2 removed)");
  Serial.println("================================================");
  Serial.print("Debounce          : "); Serial.print(DEBOUNCE_MS); Serial.println(" ms");
  Serial.print("Recovery (Task2)  : "); Serial.print(RECOVERY_MS); Serial.println(" ms");
  Serial.print("Comm loss thresh  : "); Serial.print(COMM_LOSS_THRESHOLD_MS); Serial.println(" ms");
  Serial.print("Relay mismatch    : "); Serial.print(RELAY_MISMATCH_PERSIST_MS); Serial.println(" ms persistence");
  Serial.print("FAILSAFE->SHUTDOWN: "); Serial.print(FAILSAFE_TO_SHUTDOWN_TIMEOUT_MS); Serial.println(" ms timeout");
  Serial.println("bms = core-1-only, no mutex. bmsShared = cross-core, always mutex-guarded.");
  Serial.println("================================================");
}


// ============================================================
// LOOP - CORE 1 - SAFETY CRITICAL PATH ONLY
// ============================================================

void loop()
{
  // TASK 1
  runBatteryEngine();

  // TASK 2
  evaluateSafety();

  // Do not allow relay recovery after supervisory SHUTDOWN.
  if (systemState != SYS_SHUTDOWN)
  {
    updateRelayStateMachine();
  }

  
  // TASK 4
  updateSystemStateMachine();

  // Publish one coherent battery + FSM snapshot for Core 0 telemetry.
  if (xSemaphoreTake(bmsMutex, 0) == pdTRUE) {
    task56Shared.battery = bms;
    task56Shared.system = (uint8_t)systemState;
    task56Shared.relay = (uint8_t)relayState;
    task56Shared.fault = (uint8_t)activeFaultID;
    task56Shared.faultSource = (uint8_t)activeFaultSource;
    task56Shared.capturedMs = millis();
    task56SharedReady = true;
    xSemaphoreGive(bmsMutex);
  }

  // Reporting
  static unsigned long lastSerialPrint = 0;
  if (millis() - lastSerialPrint >= 1000)
  {
    lastSerialPrint = millis();
    printBatteryData();
  }

  // TASK 3
  static unsigned long lastLCDUpdate = 0;
  if (millis() - lastLCDUpdate >= CONTENT_REFRESH_INTERVAL_MS)
  {
    lastLCDUpdate = millis();
    updateLCD();
  }

  // DIAGNOSTIC FIX: loop() has no delay()/yield() anywhere else, so
  // this core's idle task never gets scheduled once core 0 has real
  // work to do (WiFi). A 1ms yield here lets FreeRTOS service the
  // watchdog and the WiFi stack without meaningfully affecting the
  // 400ms debounce / 250ms LCD timing this system relies on.
  vTaskDelay(pdMS_TO_TICKS(1));
}
