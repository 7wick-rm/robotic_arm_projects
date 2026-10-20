// Linear rail controller (ESP32 + stepper driver + AS5600 + 2 NC limit switches).
// Your original closed-loop code, fixed and extended with a SERIAL COMMAND INTERFACE so the notebook can drive it.
//
// CHANGES vs the original (all marked "FIX" / "NEW"):
//  FIX  GPIO12 is an ESP32 strapping pin: a switch that is pressed/open at reset pulls it HIGH and the board
//       fails to boot (flash-voltage select). MIN switch moved to GPIO27. REWIRE it, or set the pin back at your own risk.
//  FIX  A single failed AS5600 read faulted the whole system. Now 5 consecutive failures are needed.
//  FIX  Speed jumped 0 -> 500 pps instantly (lost steps on a loaded carriage). Added acceleration slew limit.
//  FIX  Final approach was exponential and very slow (speed = Kp*error -> a few pps). Added a minimum speed outside the deadband.
//  FIX  I2C at 50 kHz made each loop ~1.5 ms and step pulses uneven (runSpeed() is called once per loop). Default 100 kHz.
//  NEW  Stall / lost-step detection (commanded speed but the encoder does not move) -> FAULT instead of pushing on.
//  NEW  Target comes from serial commands (pot kept as optional MODE POT). Move timeout. HOME command to recover from FAULT.
//  FIX  The rail NEVER moves on its own: no homing at boot (opening the serial port resets the ESP32, which used to start a full-length
//       homing run before the arm was stowed). The host sends HOME explicitly; MOVE before HOME -> ERR NOT_HOMED.
//  FIX  STOP now aborts homing too (before, the motor kept moving during homing although STOPPED was printed).
//  NEW  Status printing is OFF by default (it would pollute the serial protocol). VERBOSE 1 turns it on for tuning.
//
// SERIAL PROTOCOL (115200, newline-terminated, case-insensitive). One reply line per command; async lines start with a keyword:
//   PING                 -> PONG
//   STATUS?              -> STATE <UNHOMED|HOMING|READY|FAULT> POS <mm> TARGET <mm> TRAVEL <mm> MODE <SERIAL|POT>
//   MOVE <mm>            -> ACK <clamped_mm>   ... later:  DONE <mm>      (position from the MIN switch, mm)
//   MOVEPCT <0..100>     -> ACK <clamped_mm>   ... later:  DONE <mm>      (percent of the measured travel)
//   STOP                 -> STOPPED <mm>       (holds the current position; no DONE for the cancelled move)
//   HOME                 -> runs the homing sequence (also clears a FAULT); progress lines, then "HOMING COMPLETE" (state READY)
//   MODE SERIAL|POT      -> MODE <x>           VERBOSE 0|1 -> VERBOSE <x>        INFO -> INFO ...
// Errors: ERR NOT_HOMED | ERR FAULT | ERR BAD_ARG | ERR UNKNOWN.   Faults print "FAULT: <reason>".
// NOTE: the ESP32 resets when the serial port is opened (DTR) and then WAITS (state UNHOMED). The Python side sends HOME before the first move.

#include <Wire.h>
#include <AccelStepper.h>
#include <stdlib.h>
#include <string.h>

// ---------- ESP32 PINS ----------
const int STEP_PIN = 25;
const int DIR_PIN  = 26;
const int SDA_PIN  = 21;
const int SCL_PIN  = 22;
const int POT_PIN  = 34;   // pot ends wired directly to 3.3V and GND (only used in MODE POT)

// Limit switches: NC pin to the GPIO, COM to GND, NO not connected. Pressed (or broken wire) reads HIGH.
const int LIMIT_MIN_NC_PIN = 27;   // FIX: was 12 (strapping pin). Base switch (position 0)
const int LIMIT_MAX_NC_PIN = 14;   // End switch (far end)

// ---------- AS5600 ----------
const uint8_t AS5600_ADDR = 0x36;
const uint8_t RAW_ANGLE_REG = 0x0C;
const int ENCODER_DIR = 1;
const uint32_t I2C_CLOCK_HZ = 100000;   // FIX: was 50000. AS5600 allows up to 1 MHz; go back to 50000 if your wiring is noisy.

// ---------- MECHANISM ----------
const float MM_PER_REV = 8.0f;
const float COUNTS_PER_MM = 4096.0f / MM_PER_REV;
const float EXPECTED_TRAVEL_MM = 250.0f;   // only for a warning print

const float BACKOFF_MM = 2.0f;
const float BACKOFF_MAX_MM = 10.0f;
const float END_MARGIN_MM = 3.0f;
const float TARGET_MARGIN_MM = 3.0f;       // targets stay this far inside both switches

const long BACKOFF_COUNTS = (long)(BACKOFF_MM * COUNTS_PER_MM);
const long BACKOFF_MAX_COUNTS = (long)(BACKOFF_MAX_MM * COUNTS_PER_MM);
const long END_MARGIN_COUNTS = (long)(END_MARGIN_MM * COUNTS_PER_MM);
const long TARGET_MARGIN_COUNTS = (long)(TARGET_MARGIN_MM * COUNTS_PER_MM);

// ---------- MOTION ----------
// Positive speed = toward MAX (End) switch, negative = toward MIN (Base) switch (position 0)
const float HOME_SPEED = 500.0f;           // steps/s. CHECK: depends on your driver microstepping (mm/s = pps / steps_per_rev * 8)
const float MAX_SPEED = 500.0f;
const float MIN_SPEED = 60.0f;             // FIX: minimum speed while outside the deadband (breakaway + linear final approach)
const float ACCEL_PPS2 = 2000.0f;          // FIX: slew limit, steps/s per second

// ---------- CONTROL ----------
const float Kp = 0.08f;                    // speed[pps] = Kp * error[counts]
const long DEADBAND_COUNTS = 8;
const long ARRIVE_TOL_COUNTS = 24;         // "DONE" when within this for ARRIVE_SETTLE_MS
const unsigned long ARRIVE_SETTLE_MS = 150;
const unsigned long PID_PERIOD_MS = 10;
const unsigned long PRINT_PERIOD_MS = 100;
const unsigned long HOME_TIMEOUT_MS = 60000UL;   // each homing phase
const unsigned long MOVE_TIMEOUT_MS = 90000UL;

// Stall detection: commanded >= STALL_MIN_PPS but the encoder moved < STALL_MIN_COUNTS in STALL_MS
const float STALL_MIN_PPS = 100.0f;
const long STALL_MIN_COUNTS = 12;
const unsigned long STALL_MS = 1500;
const int MAX_ENCODER_FAILS = 5;

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

int lastRaw = -1;
long encoderCounts = 0;
int encoderFails = 0;

enum HomeState { UNHOMED, CLEAR_MIN, SEEK_MAX, BACK_OFF_MAX, SEEK_MIN, BACK_OFF_MIN, READY, FAULT };
HomeState homeState = UNHOMED;
const bool AUTO_HOME_ON_BOOT = false;      // keep false: homing moves the rail over its full length, only do it when the arm is stowed

long maxHitCounts = 0, minHitCounts = 0, phaseStartCounts = 0, travelCounts = 0;
unsigned long homeStarted = 0, lastPID = 0, lastPrint = 0;
bool homed = false;

float filteredPot = 0.0f, targetCounts = 0.0f, errorCounts = 0.0f, outputSpeed = 0.0f, cmdSpeed = 0.0f;

bool potMode = false;          // false = target from serial commands
bool verbose = false;
bool moveActive = false;       // a MOVE is pending, DONE not yet sent
unsigned long moveStarted = 0, arrivedSince = 0;
long stallRefCounts = 0;
unsigned long stallRefMs = 0;

char cmdBuf[48];
int cmdLen = 0;

bool maxPressed() { return digitalRead(LIMIT_MAX_NC_PIN) == HIGH; }
bool minPressed() { return digitalRead(LIMIT_MIN_NC_PIN) == HIGH; }

float posMM() { return encoderCounts / COUNTS_PER_MM; }

void fault(const char *message) {
    motor.setSpeed(0);
    outputSpeed = 0;
    cmdSpeed = 0;
    homed = false;
    moveActive = false;
    homeState = FAULT;
    Serial.print("FAULT: ");
    Serial.println(message);
}

void startPhase(HomeState next, const char *message) {
    motor.setSpeed(0);
    homeState = next;
    homeStarted = millis();
    Serial.println(message);
}

bool readAngle(int &angle) {
    for (int attempt = 0; attempt < 3; attempt++) {
        Wire.beginTransmission(AS5600_ADDR);
        Wire.write(RAW_ANGLE_REG);
        if (Wire.endTransmission(false) == 0) {
            int received = Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
            if (received == 2 && Wire.available() >= 2) {
                int highByte = Wire.read();
                int lowByte = Wire.read();
                angle = ((highByte << 8) | lowByte) & 0x0FFF;
                return true;
            }
        }
        while (Wire.available()) Wire.read();
        delayMicroseconds(500);
    }
    return false;
}

bool updateEncoder() {
    int raw;
    if (!readAngle(raw)) return false;
    if (lastRaw < 0) { lastRaw = raw; return true; }
    int delta = raw - lastRaw;
    if (delta > 2048)  delta -= 4096;
    if (delta < -2048) delta += 4096;
    encoderCounts += (long)delta * ENCODER_DIR;
    lastRaw = raw;
    return true;
}

float clampTargetMM(float mm) {
    float lo = TARGET_MARGIN_MM;
    float hi = travelCounts / COUNTS_PER_MM - TARGET_MARGIN_MM;
    if (hi < lo) hi = lo;
    return constrain(mm, lo, hi);
}

void updatePot() {
    int value = analogRead(POT_PIN);
    filteredPot = 0.85f * filteredPot + 0.15f * value;
    float fraction = constrain(filteredPot / 4095.0f, 0.0f, 1.0f);
    float usable = (float)(travelCounts - 2 * TARGET_MARGIN_COUNTS);
    if (usable < 0.0f) usable = 0.0f;
    targetCounts = (float)TARGET_MARGIN_COUNTS + fraction * usable;
}

void completeHoming() {
    motor.setSpeed(0);
    long measured = maxHitCounts - minHitCounts;
    if (measured < 4 * TARGET_MARGIN_COUNTS) {
        fault("Measured travel too small or negative (check switch positions and ENCODER_DIR)");
        return;
    }
    travelCounts = measured;
    encoderCounts -= minHitCounts;   // MIN trigger point = 0, MAX trigger point = travelCounts

    Serial.printf("Measured travel: %.2f mm (expected %.1f mm)\n", travelCounts / COUNTS_PER_MM, EXPECTED_TRAVEL_MM);

    filteredPot = analogRead(POT_PIN);
    updatePot();
    if (!potMode) targetCounts = encoderCounts;   // stay where we are until the first MOVE

    errorCounts = 0; outputSpeed = 0; cmdSpeed = 0;
    lastPID = millis();
    moveActive = false;
    stallRefCounts = encoderCounts; stallRefMs = millis();
    homed = true;
    homeState = READY;
    Serial.println("HOMING COMPLETE");
}

// NEW: (re)start the homing sequence. Used at boot and by the HOME command (also recovers from FAULT).
void beginHoming() {
    lastRaw = -1;                // re-sync the encoder reference (it is not tracked while FAULT)
    motor.setSpeed(0);
    outputSpeed = 0; cmdSpeed = 0;
    homed = false;
    moveActive = false;
    homeStarted = millis();
    if (minPressed() && maxPressed()) { fault("Both limit switches pressed (wiring problem)"); return; }
    if (minPressed()) {
        phaseStartCounts = encoderCounts;
        startPhase(CLEAR_MIN, "MIN switch active. Clearing toward MAX.");
    } else {
        startPhase(SEEK_MAX, "Calibration: seeking MAX switch.");
    }
}

void runHoming() {
    if (millis() - homeStarted > HOME_TIMEOUT_MS) { fault("Homing phase timeout"); return; }
    if (minPressed() && maxPressed()) { fault("Both limit switches pressed at once (wiring problem)"); return; }

    if (homeState == CLEAR_MIN) {
        long moved = labs(encoderCounts - phaseStartCounts);
        if (moved >= BACKOFF_MAX_COUNTS && minPressed()) { fault("MIN switch did not release while clearing"); return; }
        if (!minPressed() && moved >= BACKOFF_COUNTS) { startPhase(SEEK_MAX, "MIN cleared. Seeking MAX switch."); return; }
        motor.setSpeed(HOME_SPEED); motor.runSpeed(); return;
    }
    if (homeState == SEEK_MAX) {
        if (minPressed()) { fault("MIN switch hit while seeking MAX (switches swapped or direction reversed)"); return; }
        if (maxPressed()) { maxHitCounts = encoderCounts; startPhase(BACK_OFF_MAX, "MAX detected. Backing off."); return; }
        motor.setSpeed(HOME_SPEED); motor.runSpeed(); return;
    }
    if (homeState == BACK_OFF_MAX) {
        long movedAway = labs(encoderCounts - maxHitCounts);
        if (movedAway >= BACKOFF_MAX_COUNTS && maxPressed()) { fault("MAX switch did not release during backoff"); return; }
        if (!maxPressed() && movedAway >= BACKOFF_COUNTS) { startPhase(SEEK_MIN, "MAX cleared. Seeking MIN switch."); return; }
        motor.setSpeed(-HOME_SPEED); motor.runSpeed(); return;
    }
    if (homeState == SEEK_MIN) {
        if (maxPressed()) { fault("MAX switch hit while seeking MIN"); return; }
        if (minPressed()) { minHitCounts = encoderCounts; startPhase(BACK_OFF_MIN, "MIN detected. Backing off."); return; }
        motor.setSpeed(-HOME_SPEED); motor.runSpeed(); return;
    }
    if (homeState == BACK_OFF_MIN) {
        long movedAway = labs(encoderCounts - minHitCounts);
        if (movedAway >= BACKOFF_MAX_COUNTS && minPressed()) { fault("MIN switch did not release during backoff"); return; }
        if (!minPressed() && movedAway >= BACKOFF_COUNTS) { completeHoming(); return; }
        motor.setSpeed(HOME_SPEED); motor.runSpeed(); return;
    }
}

// ---------- POSITION CONTROL ----------
void runPID() {
    unsigned long now = millis();
    if (now - lastPID < PID_PERIOD_MS) return;
    float dt = (now - lastPID) / 1000.0f;
    lastPID = now;
    if (dt <= 0.0f) return;

    if (maxPressed()) { fault("MAX limit switch activated during movement"); return; }
    if (minPressed()) { fault("MIN limit switch activated during movement"); return; }
    if (encoderCounts < -END_MARGIN_COUNTS || encoderCounts > (travelCounts + END_MARGIN_COUNTS)) {
        fault("Software travel limit exceeded"); return;
    }

    if (potMode) updatePot();

    errorCounts = targetCounts - encoderCounts;
    float desired;
    if (labs((long)errorCounts) <= DEADBAND_COUNTS) {
        desired = 0.0f;
    } else {
        desired = Kp * errorCounts;
        float mag = fabsf(desired);
        if (mag < MIN_SPEED) mag = MIN_SPEED;          // FIX
        if (mag > MAX_SPEED) mag = MAX_SPEED;
        desired = (errorCounts > 0) ? mag : -mag;
    }
    // FIX: slew limit (smooth start/stop, no lost steps)
    float maxDelta = ACCEL_PPS2 * dt;
    if (desired > cmdSpeed + maxDelta) cmdSpeed += maxDelta;
    else if (desired < cmdSpeed - maxDelta) cmdSpeed -= maxDelta;
    else cmdSpeed = desired;
    outputSpeed = cmdSpeed;
    motor.setSpeed(cmdSpeed);

    // NEW: stall detection
    if (fabsf(cmdSpeed) >= STALL_MIN_PPS) {
        if (labs(encoderCounts - stallRefCounts) >= STALL_MIN_COUNTS) { stallRefCounts = encoderCounts; stallRefMs = now; }
        else if (now - stallRefMs > STALL_MS) { fault("Stall: motor commanded but encoder not moving (jam, lost steps, or encoder fault)"); return; }
    } else {
        stallRefCounts = encoderCounts; stallRefMs = now;
    }

    // NEW: completion + timeout for serial moves
    if (moveActive) {
        if (labs((long)errorCounts) <= ARRIVE_TOL_COUNTS) {
            if (arrivedSince == 0) arrivedSince = now;
            if (now - arrivedSince >= ARRIVE_SETTLE_MS) {
                moveActive = false;
                Serial.print("DONE "); Serial.println(posMM(), 2);
            }
        } else {
            arrivedSince = 0;
        }
        if (moveActive && now - moveStarted > MOVE_TIMEOUT_MS) fault("Move timeout");
    }
}

// ---------- SERIAL COMMANDS ----------
void startMove(float mm) {
    if (homeState == FAULT) { Serial.println("ERR FAULT"); return; }
    if (!homed) { Serial.println("ERR NOT_HOMED"); return; }
    float c = clampTargetMM(mm);
    potMode = false;
    targetCounts = c * COUNTS_PER_MM;
    moveActive = true; moveStarted = millis(); arrivedSince = 0;
    stallRefCounts = encoderCounts; stallRefMs = millis();
    Serial.print("ACK "); Serial.println(c, 2);
}

void handleCommand(char *line) {
    for (char *p = line; *p; ++p) *p = toupper(*p);
    char *arg = strchr(line, ' ');
    if (arg) { *arg = 0; ++arg; }

    if (!strcmp(line, "PING")) { Serial.println("PONG"); }
    else if (!strcmp(line, "STATUS?")) {
        const char *st = (homeState == FAULT) ? "FAULT" : (homed ? "READY" : (homeState == UNHOMED ? "UNHOMED" : "HOMING"));
        Serial.printf("STATE %s POS %.2f TARGET %.2f TRAVEL %.2f MODE %s\n", st, posMM(), targetCounts / COUNTS_PER_MM,
                      travelCounts / COUNTS_PER_MM, potMode ? "POT" : "SERIAL");
    }
    else if (!strcmp(line, "INFO")) {
        Serial.printf("INFO travel_mm=%.2f counts_per_mm=%.1f margin_mm=%.1f max_pps=%.0f\n", travelCounts / COUNTS_PER_MM,
                      COUNTS_PER_MM, TARGET_MARGIN_MM, MAX_SPEED);
    }
    else if (!strcmp(line, "MOVE")) {
        if (!arg || !*arg) { Serial.println("ERR BAD_ARG"); return; }
        startMove(atof(arg));
    }
    else if (!strcmp(line, "MOVEPCT")) {
        if (!arg || !*arg) { Serial.println("ERR BAD_ARG"); return; }
        float p = constrain(atof(arg), 0.0f, 100.0f);
        startMove(p / 100.0f * (travelCounts / COUNTS_PER_MM));
    }
    else if (!strcmp(line, "STOP")) {
        motor.setSpeed(0); outputSpeed = 0; cmdSpeed = 0;
        if (homed) { targetCounts = encoderCounts; }
        else if (homeState != FAULT) { homeState = UNHOMED; }     // aborts an ongoing homing (position reference is lost)
        moveActive = false;
        Serial.print("STOPPED "); Serial.println(posMM(), 2);
    }
    else if (!strcmp(line, "HOME")) { beginHoming(); }
    else if (!strcmp(line, "MODE")) {
        if (arg && !strcmp(arg, "POT")) potMode = true;
        else if (arg && !strcmp(arg, "SERIAL")) { potMode = false; targetCounts = encoderCounts; moveActive = false; }
        else { Serial.println("ERR BAD_ARG"); return; }
        Serial.print("MODE "); Serial.println(potMode ? "POT" : "SERIAL");
    }
    else if (!strcmp(line, "VERBOSE")) { verbose = (arg && *arg == '1'); Serial.print("VERBOSE "); Serial.println(verbose ? 1 : 0); }
    else { Serial.println("ERR UNKNOWN"); }
}

void pollSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            cmdBuf[cmdLen] = 0;
            if (cmdLen > 0) handleCommand(cmdBuf);
            cmdLen = 0;
        } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        } else {
            cmdLen = 0;   // overflow: drop the line
        }
    }
}

void printStatus() {
    if (!verbose || millis() - lastPrint < PRINT_PERIOD_MS) return;
    lastPrint = millis();
    Serial.printf("DBG Pot:%4d MIN:%d MAX:%d Target:%7.2f Actual:%7.2f Err:%6.0f Speed:%6.1f state:%d\n",
                  analogRead(POT_PIN), digitalRead(LIMIT_MIN_NC_PIN), digitalRead(LIMIT_MAX_NC_PIN),
                  targetCounts / COUNTS_PER_MM, posMM(), errorCounts, outputSpeed, (int)homeState);
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\nESP32 STARTED");
    Serial.println("Rail controller (AS5600 closed loop, two-switch calibration, serial commands)");

    pinMode(POT_PIN, INPUT);
    analogReadResolution(12);
    pinMode(LIMIT_MIN_NC_PIN, INPUT_PULLUP);
    pinMode(LIMIT_MAX_NC_PIN, INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    motor.setMaxSpeed(MAX_SPEED);
    motor.setMinPulseWidth(15);

    int firstAngle;
    if (!readAngle(firstAngle)) { fault("AS5600 not detected"); return; }
    lastRaw = firstAngle;
    encoderCounts = 0;
    filteredPot = analogRead(POT_PIN);
    if (AUTO_HOME_ON_BOOT) beginHoming();
    else Serial.println("UNHOMED: waiting for HOME command (rail will not move)");
}

void loop() {
    pollSerial();                                   // always answer commands, even in FAULT

    if (homeState == FAULT) { motor.setSpeed(0); return; }

    if (!updateEncoder()) {
        if (++encoderFails >= MAX_ENCODER_FAILS) { fault("AS5600 read failed repeatedly"); return; }
    } else {
        encoderFails = 0;
    }

    if (homeState == UNHOMED) { motor.setSpeed(0); return; }   // encoder keeps being tracked above

    if (!homed) {
        runHoming();
    } else {
        runPID();
        if (homeState != FAULT) motor.runSpeed();
    }
    printStatus();
}
