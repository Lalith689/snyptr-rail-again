// ==============================================================================
// ESP32 NEMA-23 RAIL TARGET POSITIONING & TRACKING SYSTEM
// Connected to TB6600: PUL+ -> GPIO 25, DIR+ -> GPIO 32
// ==============================================================================

#define STEP_PIN 25
#define DIR_PIN  32
#define INVERT_DIR true

const float CM_PER_REV = 6.0f;
#define MICROSTEPPING 1
const float STEPS_PER_REV = 200.0f * MICROSTEPPING;
const float STEPS_PER_CM  = STEPS_PER_REV / CM_PER_REV; // 33.3333 steps/cm
const float MIN_POSITION_CM = 0.0f;
const float MAX_POSITION_CM = 174.0f;

const unsigned long BASE_CRUISE_DELAY_US = 1230;
int currentSpeedPercent = 100;
unsigned long maxSpeedDelayUs = BASE_CRUISE_DELAY_US;
unsigned long startSpeedDelayUs = 2800;
unsigned long accelSteps = 80;

enum ContinuousDriveState { DRIVE_IDLE, DRIVE_RIGHT, DRIVE_LEFT };
ContinuousDriveState driveState = DRIVE_IDLE;
unsigned long continuousSteps = 0;
long currentStepPosition = 0;
long totalStepsTravelled = 0;

float getDistanceCm() { return (float)currentStepPosition / STEPS_PER_CM; }
float getTotalDistanceTravelledCm() { return (float)totalStepsTravelled / STEPS_PER_CM; }

void setSpeedPercent(int pct) {
  if (pct < 15) pct = 15;
  if (pct > 150) pct = 150;
  currentSpeedPercent = pct;
  maxSpeedDelayUs = (unsigned long)((float)BASE_CRUISE_DELAY_US * 100.0f / (float)pct);
  startSpeedDelayUs = maxSpeedDelayUs * 2.2f;
  if (startSpeedDelayUs < maxSpeedDelayUs + 400) startSpeedDelayUs = maxSpeedDelayUs + 400;
  accelSteps = map(pct, 15, 150, 30, 90);
}

void setMotorDirection(bool forward) {
  bool pinLevel = INVERT_DIR ? !forward : forward;
  digitalWrite(DIR_PIN, pinLevel ? HIGH : LOW);
  delayMicroseconds(10);
}

void stepMotor(unsigned long stepDelayUs, bool forward) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(stepDelayUs - 15);
  if (forward) currentStepPosition++;
  else currentStepPosition--;
  totalStepsTravelled++;
}

void printTelemetry(const char* statusLabel) {
  Serial.printf("[TELEMETRY] Status: %-8s | Pos: %6.2f cm | Odometer: %7.2f cm | Steps: %ld\n",
                statusLabel, getDistanceCm(), getTotalDistanceTravelledCm(), currentStepPosition);
}

void moveToPositionCm(float targetCm) {
  if (targetCm < MIN_POSITION_CM) targetCm = MIN_POSITION_CM;
  if (targetCm > MAX_POSITION_CM) targetCm = MAX_POSITION_CM;
  long targetSteps = lround(targetCm * STEPS_PER_CM);
  long deltaSteps = targetSteps - currentStepPosition;
  if (deltaSteps == 0) {
    printTelemetry("IDLE");
    return;
  }
  bool moveForward = (deltaSteps > 0);
  unsigned long stepsToTake = abs(deltaSteps);
  setMotorDirection(moveForward);
  for (unsigned long i = 0; i < stepsToTake; i++) {
    if (Serial.available() > 0) {
      String interruptCmd = Serial.readStringUntil('\n');
      interruptCmd.trim();
      interruptCmd.toUpperCase();
      if (interruptCmd == "STOP" || interruptCmd == "S") {
        driveState = DRIVE_IDLE;
        printTelemetry("STOPPED");
        return;
      }
    }
    unsigned long currentDelay = maxSpeedDelayUs;
    if (i < accelSteps && i < stepsToTake / 2) {
      currentDelay = map(i, 0, accelSteps, startSpeedDelayUs, maxSpeedDelayUs);
    } else if (stepsToTake - i < accelSteps && i >= stepsToTake / 2) {
      currentDelay = map(stepsToTake - i, 0, accelSteps, startSpeedDelayUs, maxSpeedDelayUs);
    }
    stepMotor(currentDelay, moveForward);
  }
  printTelemetry("REACHED");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  driveState = DRIVE_IDLE;
}

void loop() {
  if (driveState != DRIVE_IDLE) {
    bool forward = (driveState == DRIVE_RIGHT);
    long maxSteps = lround(MAX_POSITION_CM * STEPS_PER_CM);
    long minSteps = lround(MIN_POSITION_CM * STEPS_PER_CM);
    if ((forward && currentStepPosition >= maxSteps) || (!forward && currentStepPosition <= minSteps)) {
      driveState = DRIVE_IDLE;
      printTelemetry("LIMIT");
    } else {
      unsigned long d = (continuousSteps < accelSteps)
                          ? map(continuousSteps, 0, accelSteps, startSpeedDelayUs, maxSpeedDelayUs)
                          : maxSpeedDelayUs;
      stepMotor(d, forward);
      continuousSteps++;
    }
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    if (input.length() == 0) return;

    if (input == "RIGHT" || input == "D") {
      setMotorDirection(true);
      driveState = DRIVE_RIGHT;
      continuousSteps = 0;
    } else if (input == "LEFT" || input == "A") {
      setMotorDirection(false);
      driveState = DRIVE_LEFT;
      continuousSteps = 0;
    } else if (input == "STOP" || input == "S") {
      driveState = DRIVE_IDLE;
      printTelemetry("STOPPED");
    } else if (input == "ZERO") {
      driveState = DRIVE_IDLE;
      currentStepPosition = 0;
      totalStepsTravelled = 0;
      printTelemetry("ZEROED");
    } else if (input == "HOME") {
      driveState = DRIVE_IDLE;
      moveToPositionCm(0.0f);
    } else if (input == "STATUS" || input == "POS") {
      printTelemetry(driveState == DRIVE_IDLE ? "IDLE" : "MOVING");
    } else if (input.startsWith("SPEED")) {
      int commaIdx = input.indexOf(',');
      String numStr = (commaIdx != -1) ? input.substring(commaIdx + 1) : input.substring(5);
      numStr.trim();
      int pct = numStr.toInt();
      if (pct > 0) setSpeedPercent(pct);
    } else if (input.startsWith("GOTO")) {
      driveState = DRIVE_IDLE;
      int commaIdx = input.indexOf(',');
      String numStr = (commaIdx != -1) ? input.substring(commaIdx + 1) : input.substring(4);
      numStr.trim();
      moveToPositionCm(numStr.toFloat());
    }
  }
}
