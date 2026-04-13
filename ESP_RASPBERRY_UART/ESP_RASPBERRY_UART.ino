#include <WiFi.h>

// =====================================================
// WIFI AP
// =====================================================
const char* AP_SSID = "ESP32_Car";
const char* AP_PASS = "1020304050";

// TCP server
WiFiServer tcpServer(8080);
WiFiClient tcpClients[4];   // hasta 4 clientes simultáneos

// =====================================================
// UART HACIA RASPBERRY PI
// =====================================================
#define UART_RX_PIN 16
#define UART_TX_PIN 17
HardwareSerial PiSerial(2);

// =====================================================
// PINES MOTORES
// AJUSTA SEGUN TU DRIVER
// =====================================================
// Motor izquierdo
#define L_IN1 47
#define L_IN2 45
#define L_PWM 48

// Motor derecho
#define R_IN1 12
#define R_IN2 13
#define R_PWM 14

// PWM
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;
const int L_PWM_CH = 0;
const int R_PWM_CH = 1;

// =====================================================
// MODOS
// =====================================================
enum ControlMode {
  MODE_MANUAL = 0,
  MODE_AUTO   = 1
};

ControlMode currentMode = MODE_MANUAL;

String modeToString(ControlMode mode) {
  return (mode == MODE_MANUAL) ? "MANUAL" : "AUTO";
}

// =====================================================
// ESTADO DE MOVIMIENTO
// =====================================================
int targetSpeed = 0;   // -255 a 255
int targetTurn  = 0;   // -255 a 255

int currentLeft  = 0;
int currentRight = 0;

unsigned long lastManualCmdMs = 0;
unsigned long lastAutoCmdMs   = 0;
unsigned long lastAnyCmdMs    = 0;
unsigned long lastStatusMs    = 0;
unsigned long lastRampMs      = 0;

// Timeouts
const unsigned long MANUAL_TIMEOUT_MS = 700;
const unsigned long AUTO_TIMEOUT_MS   = 700;
const unsigned long GLOBAL_STOP_MS    = 1200;

// Estado periódico
const unsigned long STATUS_INTERVAL_MS = 250;

// Rampa
const unsigned long RAMP_INTERVAL_MS = 20;
const int RAMP_STEP = 8;

// Buffers de entrada
String uartBuffer = "";
String tcpBuffers[4];

// =====================================================
// UTILIDADES
// =====================================================
int clamp255(int v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

String buildStatusMessage() {
  String msg = "STATUS,";
  msg += modeToString(currentMode);
  msg += ",";
  msg += String(targetSpeed);
  msg += ",";
  msg += String(targetTurn);
  msg += ",";
  msg += String(currentLeft);
  msg += ",";
  msg += String(currentRight);
  return msg;
}

void sendToAllTcpClients(const String& msg) {
  for (int i = 0; i < 4; i++) {
    if (tcpClients[i] && tcpClients[i].connected()) {
      tcpClients[i].println(msg);
    }
  }
}

void publishStatus() {
  String msg = buildStatusMessage();
  Serial.println(msg);
  PiSerial.println(msg);
  sendToAllTcpClients(msg);
}

void publishInfo(const String& msg) {
  Serial.println(msg);
  PiSerial.println(msg);
  sendToAllTcpClients(msg);
}

// =====================================================
// CONTROL DE MOTORES
// =====================================================
void setMotorRaw(int in1, int in2, int pwmChannel, int value) {
  value = clamp255(value);

  if (value > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, value);
  } else if (value < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, -value);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

void applyMotorOutputs(int leftValue, int rightValue) {
  setMotorRaw(L_IN1, L_IN2, L_PWM_CH, leftValue);
  setMotorRaw(R_IN1, R_IN2, R_PWM_CH, rightValue);
}

void stopMotorsImmediate() {
  targetSpeed = 0;
  targetTurn  = 0;
  currentLeft = 0;
  currentRight = 0;
  applyMotorOutputs(0, 0);
}

void computeTargetsToWheels(int speedVal, int turnVal, int& leftTarget, int& rightTarget) {
  leftTarget  = clamp255(speedVal - turnVal);
  rightTarget = clamp255(speedVal + turnVal);
}

int rampTowards(int current, int target, int step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

void updateRamp() {
  unsigned long now = millis();
  if (now - lastRampMs < RAMP_INTERVAL_MS) return;
  lastRampMs = now;

  int leftTarget, rightTarget;
  computeTargetsToWheels(targetSpeed, targetTurn, leftTarget, rightTarget);

  currentLeft  = rampTowards(currentLeft, leftTarget, RAMP_STEP);
  currentRight = rampTowards(currentRight, rightTarget, RAMP_STEP);

  applyMotorOutputs(currentLeft, currentRight);
}

// =====================================================
// PARSER DE COMANDOS
// =====================================================
void handleCommand(String cmd, const String& sourceName) {
  cmd.trim();
  if (cmd.length() == 0) return;

  publishInfo("RX," + sourceName + "," + cmd);

  if (cmd.equalsIgnoreCase("STOP")) {
    targetSpeed = 0;
    targetTurn = 0;
    lastAnyCmdMs = millis();

    if (sourceName == "TCP") {
      currentMode = MODE_MANUAL;
      lastManualCmdMs = millis();
    } else if (sourceName == "UART") {
      lastAutoCmdMs = millis();
    }

    publishInfo("INFO,STOP_APPLIED");
    return;
  }

  if (cmd.equalsIgnoreCase("PING")) {
    publishInfo("PONG");
    return;
  }

  if (cmd.startsWith("MODE,")) {
    int comma = cmd.indexOf(',');
    String modeStr = cmd.substring(comma + 1);
    modeStr.trim();

    if (modeStr.equalsIgnoreCase("MANUAL")) {
      currentMode = MODE_MANUAL;
      lastManualCmdMs = millis();
      lastAnyCmdMs = millis();
      publishInfo("INFO,MODE,MANUAL");
    } else if (modeStr.equalsIgnoreCase("AUTO")) {
      currentMode = MODE_AUTO;
      lastAutoCmdMs = millis();
      lastAnyCmdMs = millis();
      publishInfo("INFO,MODE,AUTO");
    } else {
      publishInfo("ERR,UNKNOWN_MODE");
    }
    return;
  }

  if (cmd.startsWith("CMD,")) {
    int p1 = cmd.indexOf(',');
    int p2 = cmd.indexOf(',', p1 + 1);
    int p3 = cmd.indexOf(',', p2 + 1);

    if (p1 < 0 || p2 < 0 || p3 < 0) {
      publishInfo("ERR,BAD_CMD_FORMAT");
      return;
    }

    String modeStr  = cmd.substring(p1 + 1, p2);
    String speedStr = cmd.substring(p2 + 1, p3);
    String turnStr  = cmd.substring(p3 + 1);

    modeStr.trim();
    speedStr.trim();
    turnStr.trim();

    int speedVal = clamp255(speedStr.toInt());
    int turnVal  = clamp255(turnStr.toInt());

    if (modeStr.equalsIgnoreCase("MANUAL")) {
      currentMode = MODE_MANUAL;
      targetSpeed = speedVal;
      targetTurn  = turnVal;
      lastManualCmdMs = millis();
      lastAnyCmdMs = millis();
      publishInfo("INFO,APPLIED,MANUAL," + String(speedVal) + "," + String(turnVal));
      return;
    }

    if (modeStr.equalsIgnoreCase("AUTO")) {
      unsigned long now = millis();
      bool manualStillActive = (now - lastManualCmdMs) < MANUAL_TIMEOUT_MS;

      if (currentMode == MODE_MANUAL && manualStillActive && sourceName == "UART") {
        lastAutoCmdMs = now;
        lastAnyCmdMs = now;
        publishInfo("INFO,AUTO_IGNORED_MANUAL_ACTIVE");
        return;
      }

      currentMode = MODE_AUTO;
      targetSpeed = speedVal;
      targetTurn  = turnVal;
      lastAutoCmdMs = millis();
      lastAnyCmdMs = millis();
      publishInfo("INFO,APPLIED,AUTO," + String(speedVal) + "," + String(turnVal));
      return;
    }

    publishInfo("ERR,UNKNOWN_CMD_MODE");
    return;
  }

  publishInfo("ERR,UNKNOWN_COMMAND");
}

// =====================================================
// TCP
// =====================================================
void acceptNewClients() {
  if (tcpServer.hasClient()) {
    WiFiClient newClient = tcpServer.available();

    int freeSlot = -1;
    for (int i = 0; i < 4; i++) {
      if (!tcpClients[i] || !tcpClients[i].connected()) {
        freeSlot = i;
        break;
      }
    }

    if (freeSlot >= 0) {
      if (tcpClients[freeSlot]) {
        tcpClients[freeSlot].stop();
      }

      tcpClients[freeSlot] = newClient;
      tcpBuffers[freeSlot] = "";

      tcpClients[freeSlot].println("INFO,TCP_CONNECTED");
      tcpClients[freeSlot].println(buildStatusMessage());
      publishInfo("INFO,TCP_CLIENT_CONNECTED," + String(freeSlot));
    } else {
      newClient.println("ERR,NO_FREE_SLOT");
      newClient.stop();
    }
  }
}

void cleanupClients() {
  for (int i = 0; i < 4; i++) {
    if (tcpClients[i] && !tcpClients[i].connected()) {
      tcpClients[i].stop();
      tcpBuffers[i] = "";
    }
  }
}

void readTcpCommands() {
  for (int i = 0; i < 4; i++) {
    if (tcpClients[i] && tcpClients[i].connected()) {
      while (tcpClients[i].available()) {
        char c = (char)tcpClients[i].read();

        if (c == '\n') {
          handleCommand(tcpBuffers[i], "TCP");
          tcpBuffers[i] = "";
        } else if (c != '\r') {
          tcpBuffers[i] += c;
          if (tcpBuffers[i].length() > 200) {
            tcpBuffers[i] = "";
            publishInfo("ERR,TCP_BUFFER_OVERFLOW");
          }
        }
      }
    }
  }
}

// =====================================================
// UART
// =====================================================
void readUARTCommands() {
  while (PiSerial.available()) {
    char c = (char)PiSerial.read();

    if (c == '\n') {
      handleCommand(uartBuffer, "UART");
      uartBuffer = "";
    } else if (c != '\r') {
      uartBuffer += c;
      if (uartBuffer.length() > 200) {
        uartBuffer = "";
        publishInfo("ERR,UART_BUFFER_OVERFLOW");
      }
    }
  }
}

// =====================================================
// TIMEOUTS
// =====================================================
void handleTimeouts() {
  unsigned long now = millis();

  bool manualExpired = (now - lastManualCmdMs) > MANUAL_TIMEOUT_MS;
  bool autoExpired   = (now - lastAutoCmdMs) > AUTO_TIMEOUT_MS;
  bool globalExpired = (now - lastAnyCmdMs) > GLOBAL_STOP_MS;

  if (currentMode == MODE_MANUAL && manualExpired) {
    if (!autoExpired) {
      currentMode = MODE_AUTO;
      publishInfo("INFO,MODE_FALLBACK_TO_AUTO");
    } else {
      targetSpeed = 0;
      targetTurn  = 0;
    }
  }

  if (currentMode == MODE_AUTO && autoExpired) {
    targetSpeed = 0;
    targetTurn  = 0;
  }

  if (globalExpired) {
    targetSpeed = 0;
    targetTurn  = 0;
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("BOOTING...");

  PiSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  ledcSetup(L_PWM_CH, PWM_FREQ, PWM_RES);
  ledcSetup(R_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_PWM, L_PWM_CH);
  ledcAttachPin(R_PWM, R_PWM_CH);

  stopMotorsImmediate();

  WiFi.disconnect(true, true);
delay(500);

WiFi.mode(WIFI_OFF);
delay(500);

WiFi.mode(WIFI_AP);
delay(500);

bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
delay(1000);

if (apOk) {
  Serial.println("AP creado correctamente");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP del AP: ");
  Serial.println(WiFi.softAPIP());
} else {
  Serial.println("ERR,WIFI_AP_FAIL");
}

  tcpServer.begin();
  tcpServer.setNoDelay(true);

  unsigned long now = millis();
  lastManualCmdMs = now;
  lastAutoCmdMs   = now;
  lastAnyCmdMs    = now;
  lastStatusMs    = now;
  lastRampMs      = now;

  publishInfo("INFO,SYSTEM_READY");
  publishInfo("INFO,DEFAULT_MODE,MANUAL");
  publishInfo("INFO,TCP_PORT,8080");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  acceptNewClients();
  cleanupClients();
  readTcpCommands();
  readUARTCommands();
  handleTimeouts();
  updateRamp();

  unsigned long now = millis();
  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    publishStatus();
  }

  delay(2);
}