#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ====== WiFi AP ======
const char* ap_ssid = "ESP32_PRUEBA";
const char* ap_pass = "123456785";

// ====== Puente H ======
const int IN1 = 12, IN2 = 13, ENA = 18;   // Dirección
const int IN3 = 14, IN4 = 11, ENB = 8;    // Tracción (ENB cambiado de 19 a 8)

// ====== HC-SR04 ======
const int TRIG_R = 4,  ECHO_R = 5;
const int TRIG_L = 6,  ECHO_L = 7;
const int TRIG_B = 15, ECHO_B = 9;

// ====== LEDs intermitentes ======
const int LED_L = 10, LED_R = 21;

// ====== PWM ======
const int PWM_FREQ = 20000, PWM_RES = 8;
const int CH_STEER = 0, CH_DRIVE = 1;
int driveSpeed     = 180;
int steerSpeed     = 220;
int parkDriveSpeed = 140;

// ====== CALIBRACIÓN — esto vas a tunear ======
int T_STEER_FULL_MS         = 400;   // tope-a-tope de la dirección (PASO 4)
int T_AVANZAR_TRAS_HUECO_MS = 700;   // ms de avance tras detectar hueco
int DIST_CARRO_CM           = 30;    // ve un carro al lado si 
int DIST_HUECO_CM           = 50;    // ve hueco si >
int DIST_PARED_TRASERA_CM   = 10;    // distancia a la banqueta para parar

// ====== Estado ======
enum Mode { MANUAL, AUTO_PARK, SENSOR_TEST };
Mode mode = MANUAL;

enum ParkState {
  IDLE, BUSCAR_CARRO, BUSCAR_HUECO, AVANZAR_AL_CENTRO,
  GIRAR_RUEDAS, REVERSA, CENTRAR_FINAL, ESTACIONADO
};
ParkState parkState = IDLE;
unsigned long stateStart = 0;

int parkSide = +1;            // +1 = derecha, -1 = izquierda

// Posición estimada de la dirección
int steerPosMs   = 200;
int steerMoveDir = 0;
int steerMoveDur = 0;
unsigned long steerMoveStart = 0;

volatile bool hazardOn = false;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------- Motores ----------
void setMotor(int in1, int in2, int ch, int speed) {
  int s = constrain(abs(speed), 0, 255);
  if (speed > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  ledcWrite(ch, s);
}

void driveForward(int s = -1) { setMotor(IN3, IN4, CH_DRIVE, +(s<0?driveSpeed:s)); }
void driveReverse(int s = -1) { setMotor(IN3, IN4, CH_DRIVE, -(s<0?driveSpeed:s)); }
void driveStop()              { setMotor(IN3, IN4, CH_DRIVE, 0); }

// Manual: igual que antes — bang-bang
void steerRawLeft()  { setMotor(IN1, IN2, CH_STEER, -steerSpeed); steerMoveDir = 0; }
void steerRawRight() { setMotor(IN1, IN2, CH_STEER, +steerSpeed); steerMoveDir = 0; }
void steerRawStop()  { setMotor(IN1, IN2, CH_STEER, 0);           steerMoveDir = 0; }

// Auto: por tiempo
void steerTimed(int dir, int durMs) {
  steerMoveDir = dir;
  steerMoveDur = durMs;
  steerMoveStart = millis();
  if (dir > 0)      setMotor(IN1, IN2, CH_STEER, +steerSpeed);
  else if (dir < 0) setMotor(IN1, IN2, CH_STEER, -steerSpeed);
}
bool steerBusy() { return steerMoveDir != 0; }

void steerUpdate() {
  if (steerMoveDir == 0) return;
  unsigned long el = millis() - steerMoveStart;
  if (el >= (unsigned long)steerMoveDur) {
    steerPosMs = constrain(steerPosMs + steerMoveDir * (int)el, 0, T_STEER_FULL_MS);
    setMotor(IN1, IN2, CH_STEER, 0);
    steerMoveDir = 0;
  }
}

void steerGoTo(int targetMs) {
  targetMs = constrain(targetMs, 0, T_STEER_FULL_MS);
  int d = targetMs - steerPosMs;
  if (d == 0) return;
  steerTimed(d > 0 ? +1 : -1, abs(d));
}
void steerCenter()    { steerGoTo(T_STEER_FULL_MS / 2); }
void steerFullLeft()  { steerGoTo(0); }
void steerFullRight() { steerGoTo(T_STEER_FULL_MS); }

void steerCalibrateHome() {
  setMotor(IN1, IN2, CH_STEER, -steerSpeed);
  delay(T_STEER_FULL_MS + 200);
  setMotor(IN1, IN2, CH_STEER, 0);
  steerPosMs = 0;
  steerMoveDir = 0;
}

void fullStop() { driveStop(); steerRawStop(); }

// ---------- HC-SR04 ----------
long medirCm(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 25000);
  if (dur == 0) return 999;
  return dur * 0.0343 / 2;
}

// ---------- Hazards (FreeRTOS Core 0) ----------
void hazardTask(void *p) {
  pinMode(LED_L, OUTPUT); pinMode(LED_R, OUTPUT);
  bool on = false;
  while (true) {
    if (hazardOn) { on = !on; digitalWrite(LED_L, on); digitalWrite(LED_R, on); }
    else          { digitalWrite(LED_L, LOW); digitalWrite(LED_R, LOW); }
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

// ---------- Estacionamiento ----------
void startParking(int side) {
  parkSide = side;
  Serial.print(">>> AUTO PARK START side=");
  Serial.println(side > 0 ? "DERECHA" : "IZQUIERDA");
  mode = AUTO_PARK;
  hazardOn = true;
  parkState = BUSCAR_CARRO;
  stateStart = millis();
  steerCenter();
}

void abortParking(const char* m) {
  Serial.print("ABORT: "); Serial.println(m);
  fullStop();
  hazardOn = false;
  mode = MANUAL;
  parkState = IDLE;
}

void parkingLoop() {
  if (mode != AUTO_PARK) return;
  long dLat  = (parkSide > 0) ? medirCm(TRIG_R, ECHO_R) : medirCm(TRIG_L, ECHO_L);
  long dBack = medirCm(TRIG_B, ECHO_B);

  switch (parkState) {
    case BUSCAR_CARRO:
      if (steerBusy()) { driveStop(); break; }
      driveForward(parkDriveSpeed);
      if (dLat < DIST_CARRO_CM) {
        Serial.println("Carro 1 detectado");
        parkState = BUSCAR_HUECO; stateStart = millis();
      }
      if (millis() - stateStart > 15000) abortParking("timeout carro");
      break;

    case BUSCAR_HUECO:
      driveForward(parkDriveSpeed);
      if (dLat > DIST_HUECO_CM) {
        Serial.println("Hueco detectado");
        parkState = AVANZAR_AL_CENTRO; stateStart = millis();
      }
      if (millis() - stateStart > 10000) abortParking("timeout hueco");
      break;

    case AVANZAR_AL_CENTRO:
      driveForward(parkDriveSpeed);
      if (millis() - stateStart > (unsigned long)T_AVANZAR_TRAS_HUECO_MS) {
        driveStop();
        if (parkSide > 0) steerFullRight(); else steerFullLeft();
        parkState = GIRAR_RUEDAS; stateStart = millis();
      }
      break;

    case GIRAR_RUEDAS:
      driveStop();
      if (!steerBusy()) {
        driveReverse(parkDriveSpeed);
        parkState = REVERSA; stateStart = millis();
      }
      break;

    case REVERSA:
      if (dBack < DIST_PARED_TRASERA_CM && dBack > 1) {
        driveStop();
        steerCenter();
        parkState = CENTRAR_FINAL; stateStart = millis();
      }
      if (millis() - stateStart > 4000) {  // safety
        driveStop();
        steerCenter();
        parkState = CENTRAR_FINAL; stateStart = millis();
      }
      break;

    case CENTRAR_FINAL:
      driveStop();
      if (!steerBusy()) {
        Serial.println(">>> ESTACIONADO");
        parkState = ESTACIONADO;
        fullStop();
        hazardOn = false;
        mode = MANUAL;
      }
      break;

    default: break;
  }
}

// ---------- WebSocket ----------
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->opcode == WS_TEXT) {
      String cmd;
      for (size_t i = 0; i < len; i++) cmd += (char)data[i];
      cmd.trim();

      // Cualquier comando manual durante AUTO cancela
      if (mode == AUTO_PARK && cmd != "PR" && cmd != "PL") {
        abortParking("manual override");
      }

      if      (cmd == "PR") { if (mode == MANUAL) startParking(+1); }
      else if (cmd == "PL") { if (mode == MANUAL) startParking(-1); }
      else if (cmd == "T")  { mode = (mode == SENSOR_TEST) ? MANUAL : SENSOR_TEST; }
      else if (cmd == "H")  { hazardOn = !hazardOn; Serial.print("Hazards: "); Serial.println(hazardOn ? "ON" : "OFF"); }
      else if (cmd == "W")  driveForward();
      else if (cmd == "S")  driveReverse();
      else if (cmd == "X")  driveStop();
      else if (cmd == "A")  steerRawLeft();
      else if (cmd == "D")  steerRawRight();
      else if (cmd == "C")  steerRawStop();
    }
  }
  if (type == WS_EVT_DISCONNECT) {
    if (mode == AUTO_PARK) abortParking("disc");
    fullStop();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);

  ledcSetup(CH_STEER, PWM_FREQ, PWM_RES);
  ledcSetup(CH_DRIVE, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_STEER);
  ledcAttachPin(ENB, CH_DRIVE);

  fullStop();
  delay(400);
  steerCalibrateHome();
  steerCenter();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  xTaskCreatePinnedToCore(hazardTask, "hazard", 2048, NULL, 1, NULL, 0);
}

void loop() {
  steerUpdate();
  parkingLoop();

  // Sensor test mode
  static unsigned long tPrint = 0;
  if (mode == SENSOR_TEST && millis() - tPrint > 500) {
    tPrint = millis();
    Serial.printf("R:%ld  L:%ld  B:%ld\n",
      medirCm(TRIG_R, ECHO_R), medirCm(TRIG_L, ECHO_L), medirCm(TRIG_B, ECHO_B));
  }

  delay(10);
}