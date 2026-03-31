#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ====== DATOS DE LA RED QUE CREA EL ESP32 ======
const char* ap_ssid = "ESP32_CAR1";
const char* ap_pass = "12345678";   // mínimo 8 caracteres

// ====== Pines Puente H ======
const int IN1 = 12;   // Dirección
const int IN2 = 13;
const int ENA = 18;

const int IN3 = 14;   // Tracción
const int IN4 = 11;
const int ENB = 19;

// ====== PWM ======
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int CH_STEER = 0;
const int CH_DRIVE = 1;

int driveSpeed = 180;
int steerSpeed = 160;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setMotor(int in1, int in2, int ch, int speed) {
  int s = abs(speed);
  s = constrain(s, 0, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(ch, s);
}

void steerLeft()   { setMotor(IN1, IN2, CH_STEER, -steerSpeed); }
void steerRight()  { setMotor(IN1, IN2, CH_STEER, +steerSpeed); }
void steerCenter() { setMotor(IN1, IN2, CH_STEER, 0); }

void driveForward(){ setMotor(IN3, IN4, CH_DRIVE, +driveSpeed); }
void driveReverse(){ setMotor(IN3, IN4, CH_DRIVE, -driveSpeed); }
void driveStop()   { setMotor(IN3, IN4, CH_DRIVE, 0); }

void fullStop() {
  driveStop();
  steerCenter();
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) return;

  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->opcode == WS_TEXT) {
      String cmd;
      for (size_t i = 0; i < len; i++) cmd += (char)data[i];

      if      (cmd == "W") driveForward();
      else if (cmd == "S") driveReverse();
      else if (cmd == "X") fullStop();
      else if (cmd == "A") steerLeft();
      else if (cmd == "D") steerRight();
      else if (cmd == "C") steerCenter();
    }
  }

  if (type == WS_EVT_DISCONNECT) fullStop();
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcSetup(CH_STEER, PWM_FREQ, PWM_RES);
  ledcSetup(CH_DRIVE, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_STEER);
  ledcAttachPin(ENB, CH_DRIVE);

  fullStop();

  // ====== MODO AP ======
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);

  Serial.println("ESP32 AP listo");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP()); // SIEMPRE 192.168.4.1

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

void loop() {}
