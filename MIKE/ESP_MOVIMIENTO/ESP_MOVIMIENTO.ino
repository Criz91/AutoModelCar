#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// =========================================================
//  ESP32_CAR - AutoModelCar TMR 2026
//  Estacionamiento autonomo en bateria (izquierda o derecha)
// =========================================================

// ====== WiFi AP ======
const char* ap_ssid = "ESP32_PRUEBA";
const char* ap_pass = "123456785";

// ====== Pines ======
const int IN1 = 12, IN2 = 13, ENA = 18;   // Direccion
const int IN3 = 14, IN4 = 11, ENB = 8;    // Traccion

const int TRIG_R = 4,  ECHO_R = 5;
const int TRIG_L = 6,  ECHO_L = 7;
const int TRIG_B = 15, ECHO_B = 9;

const int LED_L = 10, LED_R = 21;

// ====== PWM ======
const int PWM_FREQ = 20000, PWM_RES = 8;
const int CH_STEER = 0, CH_DRIVE = 1;

// =========================================================
//  PARAMETROS AJUSTABLES EN VIVO (desde GUI)
// =========================================================
struct Params {
  int driveSpeed         = 180;   // PWM traccion manual
  int parkDriveSpeed     = 130;   // PWM traccion auto (mas lento)
  int steerSpeed         = 170;   // PWM direccion (mas suave)
  int tSteerFullMs       = 400;   // tope a tope de la direccion
  int steerTrimMs        = 0;     // sesgo de centro (-150..+150)
  int distCarroCm        = 35;    // ve carro si 
  int distHuecoCm        = 45;    // ve hueco si >
  int distFrenaSuaveCm   = 15;    // frenar suave atras
  int distParaYaCm       = 8;     // parar definitivamente atras
  int tAvanceInicialMs   = 2500;  // avance recto inicial (~1 m)
  int tAvanzarHuecoMs    = 700;   // ms tras detectar hueco antes de girar
  int minHuecoStableMs   = 250;   // hueco debe sostenerse este tiempo
  int maxAutoMs          = 60000; // watchdog del modo auto
};
Params P;

// =========================================================
//  ESTADO GLOBAL
// =========================================================
enum Mode { MANUAL, AUTO_PARK, SENSOR_TEST };
volatile Mode mode = MANUAL;

enum ParkState {
  IDLE, AVANCE_INICIAL, BUSCAR_CARRO, BUSCAR_HUECO, AVANZAR_AL_CENTRO,
  GIRAR_RUEDAS, REVERSA, FRENA_SUAVE, CENTRAR_FINAL, ESTACIONADO, ABORTADO
};
ParkState parkState = IDLE;
unsigned long stateStart = 0;
unsigned long autoStart  = 0;

int parkSide = +1;  // +1=derecha, -1=izquierda

// Posicion estimada de la direccion (open-loop)
int steerPosMs   = 200;
int steerMoveDir = 0;
int steerMoveDur = 0;
unsigned long steerMoveStart = 0;

// Hazards / LED de estado
volatile int hazardMode = 0;  // 0=off, 1=lento, 2=rapido, 3=fijo, 4=doble flash

// Filtros de mediana por sensor (5 muestras)
struct MedianFilter {
  long buf[5] = {999,999,999,999,999};
  int idx = 0;
  void push(long v) { buf[idx] = v; idx = (idx + 1) % 5; }
  long get() {
    long s[5]; for (int i=0;i<5;i++) s[i]=buf[i];
    for (int i=0;i<4;i++) for (int j=i+1;j<5;j++)
      if (s[i]>s[j]) { long t=s[i]; s[i]=s[j]; s[j]=t; }
    return s[2];
  }
};
MedianFilter filtR, filtL, filtB;

volatile long lastR = 999, lastL = 999, lastB = 999;

int countCarro = 0;
int countHueco = 0;
unsigned long huecoOpenStart = 0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// =========================================================
//  MOTORES
// =========================================================
void setMotor(int in1, int in2, int ch, int speed) {
  int s = constrain(abs(speed), 0, 255);
  if (speed > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  ledcWrite(ch, s);
}

void driveForward(int s = -1) { setMotor(IN3, IN4, CH_DRIVE, +(s<0?P.driveSpeed:s)); }
void driveReverse(int s = -1) { setMotor(IN3, IN4, CH_DRIVE, -(s<0?P.driveSpeed:s)); }
void driveStop()              { setMotor(IN3, IN4, CH_DRIVE, 0); }

void steerRawLeft()  { setMotor(IN1, IN2, CH_STEER, -P.steerSpeed); steerMoveDir = 0; }
void steerRawRight() { setMotor(IN1, IN2, CH_STEER, +P.steerSpeed); steerMoveDir = 0; }
void steerRawStop()  { setMotor(IN1, IN2, CH_STEER, 0);             steerMoveDir = 0; }

void steerTimed(int dir, int durMs) {
  steerMoveDir = dir;
  steerMoveDur = durMs;
  steerMoveStart = millis();
  if (dir > 0)      setMotor(IN1, IN2, CH_STEER, +P.steerSpeed);
  else if (dir < 0) setMotor(IN1, IN2, CH_STEER, -P.steerSpeed);
}
bool steerBusy() { return steerMoveDir != 0; }

void steerUpdate() {
  if (steerMoveDir == 0) return;
  unsigned long el = millis() - steerMoveStart;
  if (el >= (unsigned long)steerMoveDur) {
    steerPosMs = constrain(steerPosMs + steerMoveDir * (int)el, 0, P.tSteerFullMs);
    setMotor(IN1, IN2, CH_STEER, 0);
    steerMoveDir = 0;
  }
}

void steerGoTo(int targetMs) {
  targetMs = constrain(targetMs, 0, P.tSteerFullMs);
  int d = targetMs - steerPosMs;
  if (d == 0) return;
  steerTimed(d > 0 ? +1 : -1, abs(d));
}
void steerCenter()    { steerGoTo(P.tSteerFullMs / 2 + P.steerTrimMs); }
void steerFullLeft()  { steerGoTo(0); }
void steerFullRight() { steerGoTo(P.tSteerFullMs); }

void steerCalibrateHome() {
  setMotor(IN1, IN2, CH_STEER, -P.steerSpeed);
  delay(P.tSteerFullMs + 250);
  setMotor(IN1, IN2, CH_STEER, 0);
  steerPosMs = 0;
  steerMoveDir = 0;
}

void fullStop() { driveStop(); steerRawStop(); }

// =========================================================
//  HC-SR04
// =========================================================
long medirCm(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 25000);
  if (dur == 0) return 999;
  return dur * 0.0343 / 2;
}

void leerSensores() {
  filtR.push(medirCm(TRIG_R, ECHO_R));
  filtL.push(medirCm(TRIG_L, ECHO_L));
  filtB.push(medirCm(TRIG_B, ECHO_B));
  lastR = filtR.get();
  lastL = filtL.get();
  lastB = filtB.get();
}

// =========================================================
//  HAZARDS / LED de estado
// =========================================================
void hazardTask(void *p) {
  pinMode(LED_L, OUTPUT); pinMode(LED_R, OUTPUT);
  bool on = false;
  int subStep = 0;
  while (true) {
    int delayMs = 300;
    switch (hazardMode) {
      case 0:
        digitalWrite(LED_L, LOW); digitalWrite(LED_R, LOW);
        delayMs = 100;
        break;
      case 1:
        on = !on;
        digitalWrite(LED_L, on); digitalWrite(LED_R, on);
        delayMs = 350;
        break;
      case 2:
        on = !on;
        digitalWrite(LED_L, on); digitalWrite(LED_R, on);
        delayMs = 120;
        break;
      case 3:
        digitalWrite(LED_L, HIGH); digitalWrite(LED_R, HIGH);
        delayMs = 200;
        break;
      case 4:
        subStep = (subStep + 1) % 6;
        if (subStep == 0 || subStep == 2) {
          digitalWrite(LED_L, HIGH); digitalWrite(LED_R, HIGH);
        } else {
          digitalWrite(LED_L, LOW); digitalWrite(LED_R, LOW);
        }
        delayMs = 120;
        break;
    }
    vTaskDelay(delayMs / portTICK_PERIOD_MS);
  }
}

// 5 parpadeos rapidos bloqueantes antes de arrancar
void preflashLeds() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_L, HIGH); digitalWrite(LED_R, HIGH);
    delay(150);
    digitalWrite(LED_L, LOW); digitalWrite(LED_R, LOW);
    delay(150);
  }
}

// =========================================================
//  ESTACIONAMIENTO
// =========================================================
void startParking(int side) {
  parkSide = side;
  Serial.print(">>> AUTO PARK START side=");
  Serial.println(side > 0 ? "DERECHA" : "IZQUIERDA");

  // Pausa hazards y hace preflash
  hazardMode = 0;
  delay(50);
  preflashLeds();

  mode = AUTO_PARK;
  hazardMode = 0;  // apagados durante avance inicial
  parkState = AVANCE_INICIAL;
  stateStart = millis();
  autoStart  = millis();
  countCarro = 0;
  countHueco = 0;
  huecoOpenStart = 0;
  steerCenter();
}

void abortParking(const char* m) {
  Serial.print("ABORT: "); Serial.println(m);
  fullStop();
  hazardMode = 4;
  mode = MANUAL;
  parkState = ABORTADO;
}

void finishParking() {
  Serial.println(">>> ESTACIONADO");
  fullStop();
  hazardMode = 3;
  mode = MANUAL;
  parkState = ESTACIONADO;
}

void parkingLoop() {
  if (mode != AUTO_PARK) return;

  // Watchdog global
  if (millis() - autoStart > (unsigned long)P.maxAutoMs) {
    abortParking("watchdog");
    return;
  }

  long dLat  = (parkSide > 0) ? lastR : lastL;
  long dBack = lastB;

  switch (parkState) {

    case AVANCE_INICIAL:
      // Avanza recto sin buscar nada (el "1 metro inicial")
      if (steerBusy()) { driveStop(); break; }
      driveForward(P.parkDriveSpeed);
      if (millis() - stateStart > (unsigned long)P.tAvanceInicialMs) {
        Serial.println("Fin avance inicial -> buscar carro");
        parkState = BUSCAR_CARRO;
        stateStart = millis();
        countCarro = 0;
      }
      break;

    case BUSCAR_CARRO:
      if (steerBusy()) { driveStop(); break; }
      driveForward(P.parkDriveSpeed);
      if (dLat < P.distCarroCm && dLat > 3) countCarro++; else countCarro = 0;
      if (countCarro >= 3) {
        Serial.println("Carro 1 OK");
        parkState = BUSCAR_HUECO; stateStart = millis();
        countHueco = 0;
        huecoOpenStart = 0;
      }
      if (millis() - stateStart > 15000) abortParking("timeout carro");
      break;

    case BUSCAR_HUECO:
      driveForward(P.parkDriveSpeed);
      if (dLat > P.distHuecoCm) {
        countHueco++;
        if (countHueco == 1) huecoOpenStart = millis();
      } else {
        countHueco = 0;
        huecoOpenStart = 0;
      }
      if (countHueco >= 5 && (millis() - huecoOpenStart) >= (unsigned long)P.minHuecoStableMs) {
        Serial.println("Hueco OK");
        hazardMode = 1;  // intermitentes ON al confirmar hueco
        parkState = AVANZAR_AL_CENTRO;
        stateStart = millis();
      }
      if (millis() - stateStart > 12000) abortParking("timeout hueco");
      break;

    case AVANZAR_AL_CENTRO:
      driveForward(P.parkDriveSpeed);
      if (millis() - stateStart > (unsigned long)P.tAvanzarHuecoMs) {
        driveStop();
        if (parkSide > 0) steerFullRight(); else steerFullLeft();
        parkState = GIRAR_RUEDAS;
        stateStart = millis();
      }
      break;

    case GIRAR_RUEDAS:
      driveStop();
      if (!steerBusy()) {
        driveReverse(P.parkDriveSpeed);
        parkState = REVERSA;
        stateStart = millis();
      }
      break;

    case REVERSA:
      if (dLat < 4) { abortParking("lateral muy cerca"); break; }
      if (dBack < P.distFrenaSuaveCm && dBack > 1) {
        driveReverse(P.parkDriveSpeed * 0.6);
        parkState = FRENA_SUAVE;
        stateStart = millis();
      }
      if (millis() - stateStart > 4500) {
        driveStop();
        steerCenter();
        parkState = CENTRAR_FINAL;
        stateStart = millis();
      }
      break;

    case FRENA_SUAVE:
      if (dBack < P.distParaYaCm && dBack > 1) {
        driveStop();
        steerCenter();
        parkState = CENTRAR_FINAL;
        stateStart = millis();
      }
      if (millis() - stateStart > 1500) {
        driveStop();
        steerCenter();
        parkState = CENTRAR_FINAL;
        stateStart = millis();
      }
      break;

    case CENTRAR_FINAL:
      driveStop();
      if (!steerBusy()) finishParking();
      break;

    default: break;
  }
}

// =========================================================
//  TELEMETRIA
// =========================================================
const char* modeName() {
  switch (mode) {
    case MANUAL:      return parkState == ESTACIONADO ? "DONE" :
                             parkState == ABORTADO    ? "ABORT" : "MANUAL";
    case AUTO_PARK:   return "AUTO";
    case SENSOR_TEST: return "TEST";
  }
  return "?";
}

const char* parkStateName() {
  switch (parkState) {
    case IDLE: return "IDLE";
    case AVANCE_INICIAL: return "AVANCE_INI";
    case BUSCAR_CARRO: return "BUSCAR_CARRO";
    case BUSCAR_HUECO: return "BUSCAR_HUECO";
    case AVANZAR_AL_CENTRO: return "AVANZAR";
    case GIRAR_RUEDAS: return "GIRAR";
    case REVERSA: return "REVERSA";
    case FRENA_SUAVE: return "FRENA";
    case CENTRAR_FINAL: return "CENTRAR";
    case ESTACIONADO: return "OK";
    case ABORTADO: return "ABORT";
  }
  return "?";
}

void sendTelemetry() {
  if (ws.count() == 0) return;
  StaticJsonDocument<256> j;
  j["t"]    = "tel";
  j["mode"] = modeName();
  j["st"]   = parkStateName();
  j["R"]    = lastR;
  j["L"]    = lastL;
  j["B"]    = lastB;
  j["sp"]   = steerPosMs;
  j["side"] = parkSide;
  char buf[256];
  size_t n = serializeJson(j, buf);
  ws.textAll(buf, n);
}

// =========================================================
//  WEBSOCKET
// =========================================================
void applyParam(const String& k, int v) {
  if      (k=="driveSpeed")       P.driveSpeed       = constrain(v,0,255);
  else if (k=="parkDriveSpeed")   P.parkDriveSpeed   = constrain(v,0,255);
  else if (k=="steerSpeed")       P.steerSpeed       = constrain(v,0,255);
  else if (k=="tSteerFullMs")     P.tSteerFullMs     = constrain(v,100,1500);
  else if (k=="steerTrimMs")      P.steerTrimMs      = constrain(v,-150,150);
  else if (k=="distCarroCm")      P.distCarroCm      = constrain(v,5,100);
  else if (k=="distHuecoCm")      P.distHuecoCm      = constrain(v,10,200);
  else if (k=="distFrenaSuaveCm") P.distFrenaSuaveCm = constrain(v,3,50);
  else if (k=="distParaYaCm")     P.distParaYaCm     = constrain(v,2,30);
  else if (k=="tAvanceInicialMs") P.tAvanceInicialMs = constrain(v,0,10000);
  else if (k=="tAvanzarHuecoMs")  P.tAvanzarHuecoMs  = constrain(v,0,5000);
  else if (k=="minHuecoStableMs") P.minHuecoStableMs = constrain(v,0,2000);
  else if (k=="maxAutoMs")        P.maxAutoMs        = constrain(v,5000,300000);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->opcode != WS_TEXT) return;

    String cmd;
    for (size_t i = 0; i < len; i++) cmd += (char)data[i];
    cmd.trim();

    if (cmd.startsWith("SET:")) {
      int eq = cmd.indexOf('=');
      if (eq > 4) {
        String k = cmd.substring(4, eq);
        int v = cmd.substring(eq+1).toInt();
        applyParam(k, v);
      }
      return;
    }

    // Cualquier comando manual durante AUTO cancela
    if (mode == AUTO_PARK && cmd != "PR" && cmd != "PL") {
      abortParking("manual override");
    }

    if      (cmd == "PR")    { if (mode == MANUAL) startParking(+1); }
    else if (cmd == "PL")    { if (mode == MANUAL) startParking(-1); }
    else if (cmd == "T")     { mode = (mode == SENSOR_TEST) ? MANUAL : SENSOR_TEST; }
    else if (cmd == "H")     { hazardMode = (hazardMode == 0) ? 1 : 0; }
    else if (cmd == "CAL")   { steerCalibrateHome(); steerCenter(); }
    else if (cmd == "ESTOP") { abortParking("estop"); fullStop(); }
    else if (cmd == "W")     driveForward();
    else if (cmd == "S")     driveReverse();
    else if (cmd == "X")     driveStop();
    else if (cmd == "A")     steerRawLeft();
    else if (cmd == "D")     steerRawRight();
    else if (cmd == "C")     steerRawStop();
  }
  if (type == WS_EVT_DISCONNECT) {
    if (mode == AUTO_PARK) abortParking("disc");
    fullStop();
  }
}

// =========================================================
//  SETUP / LOOP
// =========================================================
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
  leerSensores();
  parkingLoop();

  static unsigned long tTel = 0;
  if (millis() - tTel > 200) {
    tTel = millis();
    sendTelemetry();
  }

  static unsigned long tPrint = 0;
  if (mode == SENSOR_TEST && millis() - tPrint > 500) {
    tPrint = millis();
    Serial.printf("R:%ld  L:%ld  B:%ld\n", lastR, lastL, lastB);
  }

  delay(10);
}