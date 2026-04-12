#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* AP_SSID = "ESP32_CAR_CTRL";
const char* AP_PASS = "12345678";

#define UART_RX_PIN 16
#define UART_TX_PIN 17
HardwareSerial PiSerial(2);

#define L_IN1 4
#define L_IN2 5
#define L_PWM 6

#define R_IN1 7
#define R_IN2 15
#define R_PWM 38

#define PWM_FREQ 1000
#define PWM_RES 8
#define L_PWM_CH 0
#define R_PWM_CH 1

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

enum ControlMode {
  MODE_MANUAL = 0,
  MODE_AUTO = 1
};

String modeToString(ControlMode m) {
  return (m == MODE_MANUAL) ? "MANUAL" : "AUTO";
}

ControlMode currentMode = MODE_MANUAL;

int targetSpeed = 0;
int targetTurn  = 0;
int currentLeft  = 0;
int currentRight = 0;

unsigned long lastManualCmdMs = 0;
unsigned long lastAutoCmdMs   = 0;
unsigned long lastAnyCmdMs    = 0;
unsigned long lastStatusMs    = 0;
unsigned long lastRampMs      = 0;

const unsigned long MANUAL_TIMEOUT_MS  = 700;
const unsigned long AUTO_TIMEOUT_MS    = 700;
const unsigned long GLOBAL_STOP_MS     = 1000;
const unsigned long STATUS_INTERVAL_MS = 250;
const unsigned long RAMP_INTERVAL_MS   = 20;
const int RAMP_STEP = 8;

String uartBuffer = "";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ESP32 Car Control</title>
</head>
<body>
  <h2>ESP32 Car Control</h2>
  <p id="status">Desconectado</p>
  <button onclick="sendCmd('CMD,MANUAL,120,0')">Adelante</button>
  <button onclick="sendCmd('CMD,MANUAL,-120,0')">Atras</button>
  <button onclick="sendCmd('CMD,MANUAL,0,-120')">Izquierda</button>
  <button onclick="sendCmd('CMD,MANUAL,0,120')">Derecha</button>
  <button onclick="sendCmd('STOP')">STOP</button>
  <button onclick="sendCmd('MODE,AUTO')">AUTO</button>
  <button onclick="sendCmd('MODE,MANUAL')">MANUAL</button>
  <pre id="log"></pre>
<script>
let ws;
function log(msg){
  const el = document.getElementById('log');
  el.textContent += msg + "\n";
  el.scrollTop = el.scrollHeight;
}
function connect(){
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => {
    document.getElementById('status').textContent = 'Conectado';
    log('WebSocket conectado');
  };
  ws.onclose = () => {
    document.getElementById('status').textContent = 'Desconectado';
    log('WebSocket desconectado');
    setTimeout(connect, 1000);
  };
  ws.onmessage = (event) => { log(event.data); };
}
function sendCmd(cmd){
  if(ws && ws.readyState === WebSocket.OPEN){
    ws.send(cmd);
    log('TX> ' + cmd);
  }
}
connect();
</script>
</body>
</html>
)rawliteral";

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

void publishStatus() {
  String msg = buildStatusMessage();
  Serial.println(msg);
  PiSerial.println(msg);
  ws.textAll(msg);
}

void publishInfo(const String& msg) {
  Serial.println(msg);
  PiSerial.println(msg);
  ws.textAll(msg);
}

void setMotorRaw(int in1, int in2, int pwmPin, int channel, int value) {
  value = clamp255(value);
  if (value > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(channel, value);
  } else if (value < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(channel, -value);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(channel, 0);
  }
}

void applyMotorOutputs(int leftValue, int rightValue) {
  setMotorRaw(L_IN1, L_IN2, L_PWM, L_PWM_CH, leftValue);
  setMotorRaw(R_IN1, R_IN2, R_PWM, R_PWM_CH, rightValue);
}

void stopMotorsImmediate() {
  targetSpeed  = 0;
  targetTurn   = 0;
  currentLeft  = 0;
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
  currentLeft  = rampTowards(currentLeft,  leftTarget,  RAMP_STEP);
  currentRight = rampTowards(currentRight, rightTarget, RAMP_STEP);
  applyMotorOutputs(currentLeft, currentRight);
}

void handleCommand(String cmd, const String& sourceName) {
  cmd.trim();
  if (cmd.length() == 0) return;

  publishInfo("RX," + sourceName + "," + cmd);

  if (cmd.equalsIgnoreCase("STOP")) {
    targetSpeed  = 0;
    targetTurn   = 0;
    lastAnyCmdMs = millis();
    if (sourceName == "WS") {
      currentMode     = MODE_MANUAL;
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
    String modeStr = cmd.substring(cmd.indexOf(',') + 1);
    modeStr.trim();
    if (modeStr.equalsIgnoreCase("MANUAL")) {
      currentMode     = MODE_MANUAL;
      lastManualCmdMs = millis();
      lastAnyCmdMs    = millis();
      publishInfo("INFO,MODE,MANUAL");
    } else if (modeStr.equalsIgnoreCase("AUTO")) {
      currentMode   = MODE_AUTO;
      lastAutoCmdMs = millis();
      lastAnyCmdMs  = millis();
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
    modeStr.trim(); speedStr.trim(); turnStr.trim();
    int speedVal = clamp255(speedStr.toInt());
    int turnVal  = clamp255(turnStr.toInt());

    if (modeStr.equalsIgnoreCase("MANUAL")) {
      currentMode     = MODE_MANUAL;
      targetSpeed     = speedVal;
      targetTurn      = turnVal;
      lastManualCmdMs = millis();
      lastAnyCmdMs    = millis();
      publishInfo("INFO,APPLIED,MANUAL," + String(speedVal) + "," + String(turnVal));
      return;
    }

    if (modeStr.equalsIgnoreCase("AUTO")) {
      unsigned long now = millis();
      bool manualStillActive = (now - lastManualCmdMs) < MANUAL_TIMEOUT_MS;
      if (currentMode == MODE_MANUAL && manualStillActive && sourceName == "UART") {
        publishInfo("INFO,AUTO_IGNORED_MANUAL_ACTIVE");
        lastAutoCmdMs = now;
        lastAnyCmdMs  = now;
        return;
      }
      currentMode   = MODE_AUTO;
      targetSpeed   = speedVal;
      targetTurn    = turnVal;
      lastAutoCmdMs = millis();
      lastAnyCmdMs  = millis();
      publishInfo("INFO,APPLIED,AUTO," + String(speedVal) + "," + String(turnVal));
      return;
    }
    publishInfo("ERR,UNKNOWN_CMD_MODE");
    return;
  }
  publishInfo("ERR,UNKNOWN_COMMAND");
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    publishInfo("INFO,WS_CLIENT_CONNECTED," + String(client->id()));
    client->text(buildStatusMessage());
  } else if (type == WS_EVT_DISCONNECT) {
    publishInfo("INFO,WS_CLIENT_DISCONNECTED," + String(client->id()));
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String cmd = "";
      for (size_t i = 0; i < len; i++) cmd += (char)data[i];
      handleCommand(cmd, "WS");
    }
  }
}

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

void handleTimeouts() {
  unsigned long now = millis();
  bool manualExpired = (now - lastManualCmdMs) > MANUAL_TIMEOUT_MS;
  bool autoExpired   = (now - lastAutoCmdMs)   > AUTO_TIMEOUT_MS;
  bool globalExpired = (now - lastAnyCmdMs)    > GLOBAL_STOP_MS;

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

void setup() {
  Serial.begin(115200);
  delay(300);

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

  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
  if (apOk) {
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("ERROR: fallo al crear AP");
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", buildStatusMessage());
  });

  server.begin();

  unsigned long now = millis();
  lastManualCmdMs = now;
  lastAutoCmdMs   = now;
  lastAnyCmdMs    = now;
  lastStatusMs    = now;
  lastRampMs      = now;

  publishInfo("INFO,SYSTEM_READY");
  publishInfo("INFO,DEFAULT_MODE,MANUAL");
}

void loop() {
  readUARTCommands();
  handleTimeouts();
  updateRamp();

  unsigned long now = millis();
  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    publishStatus();
  }

  ws.cleanupClients();
}