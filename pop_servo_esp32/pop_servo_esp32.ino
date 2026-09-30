#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================================
// BUILT-IN PCA9685 I2C SERVO DRIVER (Zero External Library Dependencies!)
// Uses standard ESP32 <Wire.h> on SDA = GPIO 21, SCL = GPIO 22
// ============================================================================
struct BuiltinPCA9685 {
  uint8_t _i2caddr;
  BuiltinPCA9685(uint8_t addr = 0x40) : _i2caddr(addr) {}

  void write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  uint8_t read8(uint8_t reg) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
  }

  void begin() {
    write8(0x00, 0x00); // MODE1 reset
    delay(10);
  }

  void setPWMFreq(float freq) {
    float prescaleval = 25000000.0f / (4096.0f * freq) - 1.0f;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);
    uint8_t oldmode = read8(0x00);
    uint8_t newmode = (oldmode & 0x7F) | 0x10;
    write8(0x00, newmode);
    write8(0xFE, prescale);
    write8(0x00, oldmode);
    delay(5);
    write8(0x00, oldmode | 0xA0);
  }

  void setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(0x06 + 4 * channel);
    Wire.write(on & 0xFF);
    Wire.write(on >> 8);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
  }
};

const char* WIFI_SSID = "ESP32_Camera";
const char* WIFI_PASS = "password123";

WebServer server(80);
BuiltinPCA9685 pwm(0x40);

// UART2 pins connected to ESP32-P4
#define P4_UART_RX_PIN 16
#define P4_UART_TX_PIN 17

// Servo positions on PCA9685 (50Hz)
const int SERVO_DOWN = 350;
const int SERVO_UP   = 150;
const unsigned long DEFAULT_POP_DURATION_MS = 5000; // 5.0 Seconds standalone cycle

int currentPos[8] = {350, 350, 350, 350, 350, 350, 350, 350};
String targetStateStr = "DOWN";

// Live Cycle & Hit Telemetry State
volatile bool g_hitActive = false;
volatile unsigned long g_hitTimestampMs = 0;
volatile unsigned long g_targetUpTimestampMs = 0;
volatile unsigned long g_targetDurationMs = DEFAULT_POP_DURATION_MS;
volatile unsigned long g_reactionTimeMs = 0;
volatile int g_hitX = 400;
volatile int g_hitY = 400;
volatile int g_hitPixels = 0;
volatile uint32_t g_shotSeq = 0;
String g_lastOutcome = "READY"; // "ACTIVE", "HIT", "MISS", "READY"

// Stepper Rail Pins (TB6600: PUL+ -> GPIO 25, DIR+ -> GPIO 32)
#define STEP_PIN 25
#define DIR_PIN  32
#define INVERT_DIR true
const float STEPS_PER_CM = 200.0f / 6.0f; // 33.3333 steps/cm
const float MAX_POSITION_CM = 174.0f;
long currentStepPosition = 0;
enum DriveState { DRIVE_IDLE, DRIVE_RIGHT, DRIVE_LEFT, DRIVE_TARGET };
DriveState railDriveState = DRIVE_IDLE;
long targetStepPosition = 0;
unsigned long lastStepMicros = 0;
unsigned long stepIntervalMicros = 1230;

// Drive PCA9685 servo channels (drives channels 0..6 when channel==1 so any header pin works!)
void setServoChannels(int channel, int pos) {
  if (channel <= 1) {
    for (int ch = 0; ch <= 6; ch++) {
      pwm.setPWM(ch, 0, pos);
      currentPos[ch] = pos;
    }
  } else if (channel <= 7) {
    pwm.setPWM(channel, 0, pos);
    pwm.setPWM(channel - 1, 0, pos);
    currentPos[channel] = pos;
  }
}

void moveServoSmooth(int channel, int targetPos, int stepDelay = 3) {
  int startPos = currentPos[1];
  if (startPos == targetPos) {
    setServoChannels(channel, targetPos);
    return;
  }

  int step = (targetPos > startPos) ? 4 : -4;
  if (step > 0) {
    for (int p = startPos; p <= targetPos; p += step) {
      setServoChannels(channel, p);
      delay(stepDelay);
    }
  } else {
    for (int p = startPos; p >= targetPos; p += step) {
      setServoChannels(channel, p);
      delay(stepDelay);
    }
  }
  setServoChannels(channel, targetPos);
}

void dropTargetDown(const char* outcomeReason) {
  moveServoSmooth(1, SERVO_DOWN, 2);
  targetStateStr = "DOWN";
  g_lastOutcome = String(outcomeReason);

  // Notify ESP32-P4 and Dashboard Serial that target is now down
  Serial2.println("DOWN,1");
  Serial.println("DOWN_CONFIRMED,1");
  Serial.printf("ACK:DOWN,1,REASON=%s,SEQ=%lu\n", outcomeReason, (unsigned long)g_shotSeq);
}

void popTargetUp(int channel, unsigned long durationMs) {
  g_hitActive = false;
  g_targetDurationMs = (durationMs >= 500 && durationMs <= 60000) ? durationMs : DEFAULT_POP_DURATION_MS;
  moveServoSmooth(channel, SERVO_UP, 3);
  targetStateStr = "UP";
  g_targetUpTimestampMs = millis();
  g_lastOutcome = "ACTIVE";

  // Arm ESP32-P4 camera optimal-area detector
  Serial2.println("ARM,1");
  Serial.println("UP_CONFIRMED,1");
  Serial.printf("ACK:UP,%d,DURATION_MS=%lu\n", channel, g_targetDurationMs);
}

void executeCommand(String line, bool fromP4 = false) {
  line.trim();
  if (line.length() == 0) return;

  // Ignore raw "DOWN,1" echo from P4 so we only process validated "HIT,1,x,y,pixels"
  if (fromP4 && line.startsWith("DOWN,")) {
    return;
  }

  // 1. Optimal-Area HIT Telemetry from ESP32-P4: "HIT,1,x,y,pixels"
  if (line.startsWith("HIT,")) {
    int parsedX = 400, parsedY = 400, parsedPixels = 75;
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);
    if (c2 > 0 && c3 > 0) {
      parsedX = line.substring(c2 + 1, c3).toInt();
      if (c4 > 0) {
        parsedY = line.substring(c3 + 1, c4).toInt();
        parsedPixels = line.substring(c4 + 1).toInt();
      } else {
        parsedY = line.substring(c3 + 1).toInt();
      }
    }

    // Verify coordinates are inside the optimal area (center 400,400, radius <= 275px)
    long dx = parsedX - 400;
    long dy = parsedY - 400;
    long distSq = dx * dx + dy * dy;
    if (distSq <= (275L * 275L)) {
      g_hitX = parsedX;
      g_hitY = parsedY;
      g_hitPixels = (parsedPixels > 0) ? parsedPixels : 75;
      g_hitActive = true;
      g_hitTimestampMs = millis();
      g_reactionTimeMs = (targetStateStr == "UP" && g_targetUpTimestampMs > 0)
                           ? (millis() - g_targetUpTimestampMs)
                           : 0;
      g_shotSeq++;

      // IMMEDIATELY fold target down using servo motor!
      dropTargetDown("HIT");
    }
    return;
  }

  // 2. Pop-Up Target Commands: "0", "UP", "UP,1", "POP,1", "POP,1,5000"
  if (line == "0" || line.startsWith("UP") || line.startsWith("POP")) {
    int channel = 1;
    unsigned long durMs = DEFAULT_POP_DURATION_MS;
    int c1 = line.indexOf(',');
    int c2 = (c1 > 0) ? line.indexOf(',', c1 + 1) : -1;
    if (c1 > 0) {
      channel = line.substring(c1 + 1, (c2 > 0) ? c2 : line.length()).toInt();
      if (channel < 1 || channel > 6) channel = 1;
    }
    if (c2 > 0) {
      unsigned long parsedDur = line.substring(c2 + 1).toInt();
      if (parsedDur >= 500) durMs = parsedDur;
    }
    // Always use 5000ms (5.0 seconds) for the standalone pop-up cycle!
    popTargetUp(channel, DEFAULT_POP_DURATION_MS);
    return;
  }

  // 3. Explicit Fold-Down Command: "DOWN", "DOWN,1", "DOWN,ALL"
  if (line.startsWith("DOWN")) {
    // Only fold manually if NOT currently in an active 5s cycle that just started <200ms ago
    dropTargetDown("MANUAL_DOWN");
    return;
  }

  // 4. Stepper Rail Commands: "LEFT", "RIGHT", "STOP", "HOME", "ZERO", "GOTO,cm", "TARGET,cm", "SPEED,pct"
  if (line == "LEFT" || line == "A") {
    railDriveState = DRIVE_LEFT;
  } else if (line == "RIGHT" || line == "D") {
    railDriveState = DRIVE_RIGHT;
  } else if (line == "STOP" || line == "S") {
    railDriveState = DRIVE_IDLE;
  } else if (line == "ZERO") {
    railDriveState = DRIVE_IDLE;
    currentStepPosition = 0;
  } else if (line == "HOME") {
    targetStepPosition = 0;
    railDriveState = DRIVE_TARGET;
  } else if (line.startsWith("GOTO,") || line.startsWith("TARGET,")) {
    int comma = line.indexOf(',');
    float cm = line.substring(comma + 1).toFloat();
    if (cm < 0.0f) cm = 0.0f;
    if (cm > MAX_POSITION_CM) cm = MAX_POSITION_CM;
    targetStepPosition = lround(cm * STEPS_PER_CM);
    railDriveState = DRIVE_TARGET;
  } else if (line.startsWith("SPEED,")) {
    int pct = line.substring(6).toInt();
    if (pct < 15) pct = 15;
    if (pct > 150) pct = 150;
    stepIntervalMicros = (unsigned long)(1230.0f * 100.0f / (float)pct);
  }
}

void setupWebServer() {
  server.enableCORS(true);

  server.on("/status", HTTP_GET, []() {
    String json = "{\"status\":\"online\",\"targetState\":\"" + targetStateStr +
                  "\",\"lastOutcome\":\"" + g_lastOutcome +
                  "\",\"shotSeq\":" + String(g_shotSeq) + "}";
    server.send(200, "application/json", json);
  });

  server.on("/telemetry", HTTP_GET, []() {
    unsigned long now = millis();
    bool recentHit = g_hitActive && ((now - g_hitTimestampMs) < 3500);
    if (!recentHit) g_hitActive = false;

    unsigned long elapsedUp = (targetStateStr == "UP") ? (now - g_targetUpTimestampMs) : 0;
    unsigned long remMs = (targetStateStr == "UP" && elapsedUp < g_targetDurationMs)
                            ? (g_targetDurationMs - elapsedUp)
                            : 0;
    float railCm = (float)currentStepPosition / STEPS_PER_CM;

    String json = "{";
    json += "\"status\":\"online\",";
    json += "\"targetState\":\"" + targetStateStr + "\",";
    json += "\"lastOutcome\":\"" + g_lastOutcome + "\",";
    json += "\"shotSeq\":" + String(g_shotSeq) + ",";
    json += "\"hit\":" + String(recentHit ? "true" : "false") + ",";
    json += "\"x\":" + String(g_hitX) + ",";
    json += "\"y\":" + String(g_hitY) + ",";
    json += "\"dartPixels\":" + String(g_hitPixels) + ",";
    json += "\"reactionMs\":" + String(g_reactionTimeMs) + ",";
    json += "\"remainingMs\":" + String(remMs) + ",";
    json += "\"railCm\":" + String(railCm, 1);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/hit", HTTP_GET, []() {
    if (server.hasArg("x")) g_hitX = server.arg("x").toInt();
    if (server.hasArg("y")) g_hitY = server.arg("y").toInt();
    g_hitPixels = server.hasArg("p") ? server.arg("p").toInt() : 80;
    g_hitActive = true;
    g_hitTimestampMs = millis();
    g_reactionTimeMs = (targetStateStr == "UP") ? (millis() - g_targetUpTimestampMs) : 0;
    g_shotSeq++;
    dropTargetDown("HIT");
    server.send(200, "application/json", "{\"ok\":true,\"targetState\":\"DOWN\",\"lastOutcome\":\"HIT\"}");
  });

  // Accept ?action=UP,1 (Prarthana's dashboard), ?c=UP,1, ?cmd=UP,1, or POST body!
  server.on("/cmd", HTTP_ANY, []() {
    String cmd = "";
    if (server.hasArg("action")) cmd = server.arg("action");
    else if (server.hasArg("c")) cmd = server.arg("c");
    else if (server.hasArg("cmd")) cmd = server.arg("cmd");
    else if (server.hasArg("plain")) cmd = server.arg("plain");
    else if (server.args() > 0) cmd = server.arg(0);

    if (cmd.length() > 0) {
      executeCommand(cmd, false);
      server.send(200, "text/plain", "ACK:" + cmd);
    } else {
      server.send(400, "text/plain", "ERR:MISSING_CMD");
    }
  });

  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {
      server.send(204);
    } else {
      server.send(404, "text/plain", "Not Found");
    }
  });

  server.begin();
}

void stepMotorOnce(bool forward) {
  bool pinLevel = INVERT_DIR ? !forward : forward;
  digitalWrite(DIR_PIN, pinLevel ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, LOW);
  currentStepPosition += forward ? 1 : -1;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout reset during servo pop

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, P4_UART_RX_PIN, P4_UART_TX_PIN);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(50);

  for (int ch = 0; ch <= 6; ch++) {
    pwm.setPWM(ch, 0, SERVO_DOWN);
    currentPos[ch] = SERVO_DOWN;
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(WIFI_SSID, WIFI_PASS, 1, 0, 4);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  digitalWrite(2, HIGH); // Solid Blue LED = Wi-Fi AP Active

  setupWebServer();
  Serial.println("STATUS: READY");
  Serial.println("=== SNYPTR POP-UP & WIFI CONTROLLER READY (192.168.4.1) ===");
}

void loop() {
  server.handleClient();

  // 1. STRICT 5.0-SECOND HARDWARE AUTO-FALLBACK TIMER WHEN TARGET IS UP
  if (targetStateStr == "UP") {
    unsigned long elapsed = millis() - g_targetUpTimestampMs;
    if (elapsed >= g_targetDurationMs) {
      g_shotSeq++;
      dropTargetDown("MISS");
      Serial.printf("⏱️ 5.0s TIMEOUT EXPIRED (%lums) -> No optimal hit detected. Target fell back!\n", elapsed);
    }
  }

  // 2. Read live HIT telemetry from ESP32-P4 over UART2 (GPIO 16)
  while (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    executeCommand(line, true);
  }

  // 3. Read commands from USB Serial0
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    executeCommand(line, false);
  }

  // 4. Non-blocking Stepper Rail Motion
  if (railDriveState != DRIVE_IDLE) {
    unsigned long nowUs = micros();
    if (nowUs - lastStepMicros >= stepIntervalMicros) {
      lastStepMicros = nowUs;
      long maxSteps = lround(MAX_POSITION_CM * STEPS_PER_CM);
      if (railDriveState == DRIVE_RIGHT) {
        if (currentStepPosition < maxSteps) stepMotorOnce(true);
        else railDriveState = DRIVE_IDLE;
      } else if (railDriveState == DRIVE_LEFT) {
        if (currentStepPosition > 0) stepMotorOnce(false);
        else railDriveState = DRIVE_IDLE;
      } else if (railDriveState == DRIVE_TARGET) {
        if (currentStepPosition < targetStepPosition) stepMotorOnce(true);
        else if (currentStepPosition > targetStepPosition) stepMotorOnce(false);
        else railDriveState = DRIVE_IDLE;
      }
    }
  }
}
