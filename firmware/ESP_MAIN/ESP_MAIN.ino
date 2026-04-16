// AutoModelCar - Firmware unificado ESP32-S3
// TMR 2026 - Categoria AutoModelCar

#include <WiFi.h>
#include <ArduinoJson.h>

const char* AP_SSID = "AutoModelCar_TMR26";
const char* AP_PASS = "tmr2026robot";

WiFiServer tcpServer(8080);
WiFiClient tcpClients[4];
String tcpBuffers[4];

// UART HACIA RASPBERRY PI
#define UART_RX_PIN 16
#define UART_TX_PIN 17
HardwareSerial PiSerial(2);
String uartBuffer = "";

// PINES DEL PUENTE H Y SENSORES
const int IN1 = 12, IN2 = 11, ENA = 8;
const int IN3 = 14, IN4 = 13, ENB = 18;

const int TRIG_R = 4,  ECHO_R = 5;
const int TRIG_L = 6,  ECHO_L = 7;
const int TRIG_B = 15, ECHO_B = 9;
const int TRIG_F = 1,  ECHO_F = 2;

const int LINE_L = 38;
const int LINE_R = 39;
const int LINE_C = 40;
#define LINE_ACTIVE_LOW 1

const int LED_L = 10, LED_R = 21;

// Focos Rojos de freno
const int LED_STOP = 41;

const int PWM_FREQ = 2000;
const int PWM_RES  = 8;
const int CH_STEER = 0;
const int CH_DRIVE = 1;

struct Params {
  int driveSpeed         = 200;
  int parkDriveSpeed     = 200;
  int steerSpeed         = 170;
  int tSteerFullMs       = 400;
  int steerTrimMs        = 0;
  int distCarroCm        = 25;
  int distHuecoCm        = 45;
  int distFrenaSuaveCm   = 15;
  int distParaYaCm       = 8;
  int tAvanceInicialMs   = 0;
  int tAvanzarHuecoMs    = 0;
  int minHuecoStableMs   = 250;
  int maxAutoMs          = 60000;

  int carLargoCm         = 29;
  int carAnchoCm         = 14;
  int carAltoCm          = 25;
  int margenHuecoCm      = 6;
  int margenBanquetaCm   = 4;
  int cmPorSegPark       = 30;
  int distInicialCm      = 100;
  int kickStartPWM       = 240;
  int kickStartMs        = 150;
  int tReversaGiroMs     = 1600;
  int lineFollowSpeed    = 150;
};
Params P;

enum Mode { MANUAL, AUTO_PARK, SENSOR_TEST, LINE_FOLLOW, TEST_PARK };
volatile Mode mode = MANUAL;

enum ParkState { IDLE, AVANCE_INICIAL, MEDIR_HUECO, AVANZAR_PASA_HUECO, STOP_Y_GIRAR, REVERSA_GIRO, STOP_Y_ENDEREZAR, REVERSA_RECTA, ESTACIONADO, ABORTADO };
ParkState parkState = IDLE;
unsigned long stateStart = 0;
unsigned long autoStart  = 0;
int parkSide = +1;

int steerPosMs   = 200;
int steerMoveDir = 0;
int steerMoveDur = 0;
unsigned long steerMoveStart = 0;

volatile int hazardMode = 0;

struct MedianFilter {
  long buf[5] = {999,999,999,999,999};
  int idx = 0;
  void push(long v) { buf[idx] = v; idx = (idx + 1) % 5; }
  long get() {
    long s[5];
    for (int i = 0; i < 5; i++) s[i] = buf[i];
    for (int i = 0; i < 4; i++) {
      for (int j = i + 1; j < 5; j++) {
        if (s[i] > s[j]) { long t = s[i]; s[i] = s[j]; s[j] = t; }
      }
    }
    return s[2];
  }
};

MedianFilter filtR, filtL, filtB, filtF;
volatile long lastR = 999, lastL = 999, lastB = 999, lastF = 999;
volatile int lnL = 0, lnR = 0, lnC = 0;

int countCarro = 0, countHueco = 0;
unsigned long huecoT0 = 0, huecoLastT = 0;
int huecoLargoCm = 0;
unsigned long kickStartT0 = 0;
bool kickStartActive = false, kickArmPending = false;

int  lfLastSteerTarget = -1;
unsigned long lfStart      = 0;
unsigned long lfCruceStart = 0; 
unsigned long lfSlowUntil  = 0; 

int  tpStep = 0, tpSide = +1, tpBlinks = 0;
unsigned long tpStepStart = 0;

unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 200;
unsigned long lastAnyCmdMs = 0;
const unsigned long GLOBAL_CMD_TIMEOUT_MS = 1500;

void sendToAllTcpClients(const String& msg) {
  for (int i = 0; i < 4; i++) {
    if (tcpClients[i] && tcpClients[i].connected()) tcpClients[i].println(msg);
  }
}

void publishLine(const String& msg) {
  Serial.println(msg);
  PiSerial.println(msg);
  sendToAllTcpClients(msg);
}

String modeToString() {
  switch (mode) {
    case MANUAL: if (parkState == ESTACIONADO) return "DONE"; if (parkState == ABORTADO) return "ABORT"; return "MANUAL";
    case AUTO_PARK:   return "AUTO";
    case SENSOR_TEST: return "TEST";
    case LINE_FOLLOW: return "LF";
    case TEST_PARK:   return "TPARK";
  }
  return "?";
}

String parkStateToString() {
  switch (parkState) {
    case IDLE:               return "IDLE";
    case AVANCE_INICIAL:     return "AVANCE_INI";
    case MEDIR_HUECO:        return "MEDIR_HUECO";
    case AVANZAR_PASA_HUECO: return "AVANZAR";
    case STOP_Y_GIRAR:       return "GIRAR";
    case REVERSA_GIRO:       return "REV_GIRO";
    case STOP_Y_ENDEREZAR:   return "ENDEREZAR";
    case REVERSA_RECTA:      return "REV_RECTA";
    case ESTACIONADO:        return "OK";
    case ABORTADO:           return "ABORT";
  }
  return "?";
}

void sendTelemetry() {
  StaticJsonDocument<384> j;
  j["t"]    = "tel";
  j["mode"] = modeToString();
  j["st"]   = parkStateToString();
  j["R"]    = lastR; j["L"] = lastL; j["B"] = lastB; j["F"] = lastF;
  j["lnL"]  = lnL; j["lnR"] = lnR; j["lnC"] = lnC;
  j["sp"]   = steerPosMs; j["side"] = parkSide;

  char buf[384]; size_t n = serializeJson(j, buf);
  Serial.println(buf); PiSerial.println(buf);
  for (int i = 0; i < 4; i++) {
    if (tcpClients[i] && tcpClients[i].connected()) {
      tcpClients[i].write((const uint8_t*)buf, n); tcpClients[i].write('\n');
    }
  }
}

void setMotor(int in1, int in2, int ch, int speed) {
  int s = constrain(abs(speed), 0, 255);
  if (speed > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); } 
  else if (speed < 0) { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); } 
  else { digitalWrite(in1, LOW); digitalWrite(in2, LOW); }
  ledcWrite(ch, s);
}

inline void stopLedOn()  { digitalWrite(LED_STOP, HIGH); }
inline void stopLedOff() { digitalWrite(LED_STOP, LOW);  }

void driveForward(int s = -1) { stopLedOff(); setMotor(IN3, IN4, CH_DRIVE, +(s < 0 ? P.driveSpeed : s)); }
void driveReverse(int s = -1) { stopLedOff(); setMotor(IN3, IN4, CH_DRIVE, -(s < 0 ? P.driveSpeed : s)); }
void driveStop()              { stopLedOn();  setMotor(IN3, IN4, CH_DRIVE, 0); }

void steerRawLeft()  { setMotor(IN1, IN2, CH_STEER, -P.steerSpeed); steerMoveDir = 0; }
void steerRawRight() { setMotor(IN1, IN2, CH_STEER, +P.steerSpeed); steerMoveDir = 0; }
void steerRawStop()  { setMotor(IN1, IN2, CH_STEER, 0);             steerMoveDir = 0; }

void steerTimed(int dir, int durMs) {
  steerMoveDir = dir; steerMoveDur = durMs; steerMoveStart = millis();
  if (dir > 0)      setMotor(IN1, IN2, CH_STEER, +P.steerSpeed);
  else if (dir < 0) setMotor(IN1, IN2, CH_STEER, -P.steerSpeed);
}

bool steerBusy() { return steerMoveDir != 0; }

void steerUpdate() {
  if (steerMoveDir == 0) return;
  unsigned long el = millis() - steerMoveStart;
  if (el >= (unsigned long)steerMoveDur) {
    steerPosMs = constrain(steerPosMs + steerMoveDir * steerMoveDur, 0, P.tSteerFullMs);
    setMotor(IN1, IN2, CH_STEER, 0); steerMoveDir = 0;
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
  steerPosMs = 0; steerMoveDir = 0;
}

void fullStop() { driveStop(); steerRawStop(); }

void armKickStart() { kickStartT0 = millis(); kickStartActive = true; }

void driveForwardKick(int cruise) {
  int s;
  if (kickStartActive && (millis() - kickStartT0) < (unsigned long)P.kickStartMs) s = P.kickStartPWM;
  else { kickStartActive = false; s = cruise; }
  stopLedOff(); setMotor(IN3, IN4, CH_DRIVE, +s);
}

void driveReverseKick(int cruise) {
  int s;
  if (kickStartActive && (millis() - kickStartT0) < (unsigned long)P.kickStartMs) s = P.kickStartPWM;
  else { kickStartActive = false; s = cruise; }
  stopLedOff(); setMotor(IN3, IN4, CH_DRIVE, -s);
}

long medirCm(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 25000);
  if (dur == 0) return 999;
  return dur * 0.0343 / 2;
}

void pushSiValido(MedianFilter& f, long v) { if (v >= 5 && v < 999) f.push(v); }

void leerSensores() {
  pushSiValido(filtR, medirCm(TRIG_R, ECHO_R)); pushSiValido(filtL, medirCm(TRIG_L, ECHO_L));
  pushSiValido(filtB, medirCm(TRIG_B, ECHO_B)); pushSiValido(filtF, medirCm(TRIG_F, ECHO_F));
  lastR = filtR.get(); lastL = filtL.get(); lastB = filtB.get(); lastF = filtF.get();
}

inline int readLineSensor(int pin) {
  int raw = digitalRead(pin);
#if LINE_ACTIVE_LOW
  return (raw == LOW) ? 1 : 0;
#else
  return (raw == HIGH) ? 1 : 0;
#endif
}

void leerLineas() { lnL = readLineSensor(LINE_L); lnR = readLineSensor(LINE_R); lnC = readLineSensor(LINE_C); }

void hazardTask(void *p) {
  pinMode(LED_L, OUTPUT); pinMode(LED_R, OUTPUT);
  bool on = false; int subStep = 0;
  while (true) {
    int delayMs = 300;
    switch (hazardMode) {
      case 0: digitalWrite(LED_L, LOW); digitalWrite(LED_R, LOW); delayMs = 100; break;
      case 1: on = !on; digitalWrite(LED_L, on); digitalWrite(LED_R, on); delayMs = 350; break;
      case 2: on = !on; digitalWrite(LED_L, on); digitalWrite(LED_R, on); delayMs = 120; break;
      case 3: digitalWrite(LED_L, HIGH); digitalWrite(LED_R, HIGH); delayMs = 200; break;
      case 4:
        subStep = (subStep + 1) % 6;
        if (subStep == 0 || subStep == 2) { digitalWrite(LED_L, HIGH); digitalWrite(LED_R, HIGH); } 
        else { digitalWrite(LED_L, LOW); digitalWrite(LED_R, LOW); }
        delayMs = 120; break;
    }
    vTaskDelay(delayMs / portTICK_PERIOD_MS);
  }
}

void abortParking(const char* m) { publishLine(String("ABORT,") + m); fullStop(); hazardMode = 4; mode = MANUAL; parkState = ABORTADO; }
void finishParking() { publishLine("INFO,ESTACIONADO"); fullStop(); hazardMode = 3; mode = MANUAL; parkState = ESTACIONADO; }

int tAvanceInicialMsEff() {
  if (P.tAvanceInicialMs > 0) return P.tAvanceInicialMs;
  if (P.cmPorSegPark <= 0) return 2500;
  return P.distInicialCm * 1000 / P.cmPorSegPark;
}

int tAvanzarMedioMsEff() {
  if (P.tAvanzarHuecoMs > 0) return P.tAvanzarHuecoMs;
  if (P.cmPorSegPark <= 0) return 700;
  return (P.carLargoCm / 2 + 4) * 1000 / P.cmPorSegPark;
}

void startParking(int side) {
  parkSide = side; publishLine(String("INFO,AUTO_PARK_START,") + (side > 0 ? "RIGHT" : "LEFT"));
  mode = AUTO_PARK; hazardMode = 2; parkState = AVANCE_INICIAL;
  stateStart = millis(); autoStart  = millis();
  countCarro = 0; countHueco = 0; huecoT0 = 0; huecoLastT = 0; huecoLargoCm = 0;
  steerCenter(); kickArmPending = true;
}

void parkingLoop() {
  if (mode != AUTO_PARK) return;
  if (millis() - autoStart > (unsigned long)P.maxAutoMs) { abortParking("watchdog"); return; }
  long dLatPark  = (parkSide > 0) ? lastR : lastL;
  long dLatCalle = (parkSide > 0) ? lastL : lastR;
  long dBack     = lastB;
  int huecoMinCm = P.carAnchoCm + P.margenHuecoCm;

  switch (parkState) {
    case AVANCE_INICIAL: {
      if (steerBusy()) { driveStop(); break; }
      if (kickArmPending) { armKickStart(); kickArmPending = false; }
      driveForwardKick(P.parkDriveSpeed);
      if (millis() - stateStart > (unsigned long)tAvanceInicialMsEff()) {
        publishLine("INFO,FIN_AVANCE_INICIAL"); parkState = MEDIR_HUECO; stateStart = millis();
        huecoT0 = 0; huecoLastT = 0; huecoLargoCm = 0; countHueco = 0; countCarro = 0; hazardMode = 0;
      }
      break;
    }
    case MEDIR_HUECO: {
      driveForwardKick(P.parkDriveSpeed);
      bool enHueco = (dLatPark > P.distHuecoCm); bool enCarro = (dLatPark < P.distCarroCm);
      if (enHueco) {
        unsigned long now = millis();
        if (huecoT0 == 0) { huecoT0 = now; huecoLastT = now; huecoLargoCm = 0; countHueco = 1; } 
        else { unsigned long dt = now - huecoLastT; huecoLastT = now; huecoLargoCm += (int)(dt * (unsigned long)P.cmPorSegPark / 1000UL); countHueco++; }
        if (huecoLargoCm >= huecoMinCm && countHueco >= 3) {
          publishLine(String("INFO,HUECO_OK,") + huecoLargoCm);
          hazardMode = 1; parkState = AVANZAR_PASA_HUECO; stateStart = millis(); armKickStart(); break;
        }
      } else if (enCarro) {
        if (huecoT0 != 0) publishLine(String("INFO,HUECO_DESCARTADO,") + huecoLargoCm);
        huecoT0 = 0; huecoLastT = 0; huecoLargoCm = 0; countHueco = 0;
      }
      if (millis() - stateStart > 20000) abortParking("timeout_medir_hueco");
      break;
    }
    case AVANZAR_PASA_HUECO: {
      driveForwardKick(P.parkDriveSpeed);
      if (millis() - stateStart > (unsigned long)tAvanzarMedioMsEff()) {
        driveStop(); if (parkSide > 0) steerFullRight(); else steerFullLeft();
        parkState = STOP_Y_GIRAR; stateStart = millis();
      }
      break;
    }
    case STOP_Y_GIRAR: {
      driveStop();
      if (!steerBusy()) { publishLine("INFO,DIRECCION_GIRADA"); parkState = REVERSA_GIRO; stateStart = millis(); armKickStart(); }
      break;
    }
    case REVERSA_GIRO: {
      if (dLatPark < 4)  { abortParking("lateral_park_muy_cerca"); break; }
      if (dLatCalle < 6) { abortParking("lateral_calle_muy_cerca"); break; }
      driveReverseKick(P.parkDriveSpeed);
      if (dBack < P.distFrenaSuaveCm && dBack >= 5) {
        driveStop(); publishLine("INFO,REVERSA_GIRO_FIN_SUAVE"); parkState = STOP_Y_ENDEREZAR; stateStart = millis(); steerCenter(); break;
      }
      if (millis() - stateStart > (unsigned long)P.tReversaGiroMs) {
        driveStop(); publishLine("INFO,REVERSA_GIRO_TIMEOUT"); parkState = STOP_Y_ENDEREZAR; stateStart = millis(); steerCenter();
      }
      break;
    }
    case STOP_Y_ENDEREZAR: {
      driveStop();
      if (!steerBusy()) { publishLine("INFO,DIRECCION_CENTRADA"); parkState = REVERSA_RECTA; stateStart = millis(); armKickStart(); }
      break;
    }
    case REVERSA_RECTA: {
      if (dLatPark < 4)  { abortParking("lateral_park_muy_cerca"); break; }
      if (dLatCalle < 6) { abortParking("lateral_calle_muy_cerca"); break; }
      driveReverseKick((int)(P.parkDriveSpeed * 0.7));
      int dParo = P.distParaYaCm + P.margenBanquetaCm;
      if (dBack >= 5 && dBack < dParo) { finishParking(); break; }
      if (millis() - stateStart > 4000) { publishLine("INFO,REVERSA_RECTA_TIMEOUT"); finishParking(); }
      break;
    }
    default: break;
  }
}

void startLineFollow() {
  if (mode == AUTO_PARK) { publishLine("ERR,LF_BLOCKED_BY_AUTO_PARK"); return; }
  publishLine("INFO,LF_START");
  mode = LINE_FOLLOW; hazardMode = 1; lfStart = millis();
  lfLastSteerTarget = -1; lfCruceStart = 0; lfSlowUntil  = 0; armKickStart();
}

void stopLineFollow(const char* reason) {
  publishLine(String("INFO,LF_STOP,") + reason);
  driveStop(); hazardMode = 0; mode = MANUAL; lfLastSteerTarget = -1;
}

void lineFollowLoop() {
  if (mode != LINE_FOLLOW) return;
  if (millis() - lfStart > 30000UL) { stopLineFollow("watchdog_30s"); return; }

  if (lfCruceStart > 0) {
    driveStop(); 
    if (millis() - lfCruceStart >= 10000UL) {
      hazardMode = 1; lfCruceStart = 0; lfStart = millis(); armKickStart(); lfLastSteerTarget = -1;
      publishLine("INFO,LF_CRUCE_FIN,REANUDANDO");
    }
    return;
  }

  int curSpeed = P.lineFollowSpeed;
  if (millis() < lfSlowUntil) { curSpeed = P.lineFollowSpeed * 60 / 100; stopLedOn(); } 
  else { stopLedOff(); driveForwardKick(curSpeed); }
  
  if (millis() < lfSlowUntil) { setMotor(IN3, IN4, CH_DRIVE, +curSpeed); }

  int center    = P.tSteerFullMs / 2 + P.steerTrimMs;
  int leftFull  = 0; int rightFull = P.tSteerFullMs;
  int leftSoft  = (int)(P.tSteerFullMs * 0.35); int rightSoft = (int)(P.tSteerFullMs * 0.65);
  int target = lfLastSteerTarget;

  if      ( lnC && !lnL && !lnR) target = center;
  else if ( lnC &&  lnL && !lnR) target = leftSoft;
  else if ( lnC && !lnL &&  lnR) target = rightSoft;
  else if (!lnC &&  lnL && !lnR) target = leftFull;
  else if (!lnC && !lnL &&  lnR) target = rightFull;
  else if (!lnC &&  lnL &&  lnR) {
    publishLine("INFO,LF_CRUCE_PEATONAL_DETECTADO");
    hazardMode = 2;
    driveStop(); 
    lfCruceStart = millis();
    lfLastSteerTarget = -1;
    return;
  }

  if (target != lfLastSteerTarget && target >= 0 && !steerBusy()) {
    bool eraRecto   = (lfLastSteerTarget == center || lfLastSteerTarget < 0);
    bool ahoraCurva = (target == leftFull || target == rightFull);
    if (!eraRecto && ahoraCurva) { lfSlowUntil = millis() + 350; publishLine("INFO,LF_CURVA"); }
    steerGoTo(target); lfLastSteerTarget = target;
  }
}

void startTestPark(int side) {
  if (mode == AUTO_PARK) { publishLine("ERR,TPARK_BLOCKED_BY_AUTO_PARK"); return; }
  tpSide = side; tpStep = 0; tpStepStart = millis(); mode = TEST_PARK; hazardMode = 0; parkState = IDLE;
  publishLine(String("INFO,TPARK_START,") + (side > 0 ? "RIGHT" : "LEFT"));
}

void testParkLoop() {
  if (mode != TEST_PARK) return;
  unsigned long elapsed = millis() - tpStepStart;
  switch (tpStep) {
    case 0: {
      driveStop();
      int blinkPhase = (int)(elapsed / 333);
      if (blinkPhase % 2 == 0) { digitalWrite(LED_L, HIGH); digitalWrite(LED_R, HIGH); } 
      else { digitalWrite(LED_L, LOW); digitalWrite(LED_R, LOW); }
      if (elapsed > 2000) {
        digitalWrite(LED_L, LOW); digitalWrite(LED_R, LOW); armKickStart();
        tpStep = 1; tpStepStart = millis(); parkState = AVANCE_INICIAL;
      }
      break;
    }
    case 1: {
      driveForwardKick(P.parkDriveSpeed);
      if (elapsed > (unsigned long)tAvanceInicialMsEff()) {
        driveStop(); hazardMode = 1; tpStep = 2; tpStepStart = millis(); parkState = AVANZAR_PASA_HUECO;
      }
      break;
    }
    case 2: {
      driveStop();
      if (elapsed > 500) { hazardMode = 0; if (tpSide > 0) steerFullLeft(); else steerFullRight(); armKickStart(); tpStep = 3; tpStepStart = millis(); parkState = STOP_Y_GIRAR; }
      break;
    }
    case 3: {
      driveForwardKick(P.parkDriveSpeed);
      if (elapsed > 1000) { driveStop(); if (tpSide > 0) steerFullRight(); else steerFullLeft(); armKickStart(); tpStep = 4; tpStepStart = millis(); parkState = REVERSA_GIRO; }
      break;
    }
    case 4: {
      driveReverseKick(P.parkDriveSpeed);
      if (elapsed > 2000) { driveStop(); steerCenter(); armKickStart(); tpStep = 5; tpStepStart = millis(); parkState = REVERSA_RECTA; }
      break;
    }
    case 5: {
      driveReverseKick((int)(P.parkDriveSpeed * 0.75));
      if (elapsed > 5000) { driveStop(); tpStep = 6; tpStepStart = millis(); parkState = ESTACIONADO; finishParking(); }
      break;
    }
  }
}

void applyParam(const String& k, int v) {
  if      (k=="driveSpeed")        P.driveSpeed        = constrain(v,0,255);
  else if (k=="parkDriveSpeed")    P.parkDriveSpeed    = constrain(v,0,255);
  else if (k=="steerSpeed")        P.steerSpeed        = constrain(v,0,255);
  else if (k=="tSteerFullMs")      P.tSteerFullMs      = constrain(v,100,1500);
  else if (k=="steerTrimMs")       P.steerTrimMs       = constrain(v,-150,150);
  else if (k=="distCarroCm")       P.distCarroCm       = constrain(v,5,100);
  else if (k=="distHuecoCm")       P.distHuecoCm       = constrain(v,10,200);
  else if (k=="distFrenaSuaveCm")  P.distFrenaSuaveCm  = constrain(v,3,50);
  else if (k=="distParaYaCm")      P.distParaYaCm      = constrain(v,2,30);
  else if (k=="tAvanceInicialMs")  P.tAvanceInicialMs  = constrain(v,0,10000);
  else if (k=="tAvanzarHuecoMs")   P.tAvanzarHuecoMs   = constrain(v,0,5000);
  else if (k=="minHuecoStableMs")  P.minHuecoStableMs  = constrain(v,0,2000);
  else if (k=="maxAutoMs")         P.maxAutoMs         = constrain(v,5000,300000);
  else if (k=="carLargoCm")        P.carLargoCm        = constrain(v,5,100);
  else if (k=="carAnchoCm")        P.carAnchoCm        = constrain(v,5,50);
  else if (k=="carAltoCm")         P.carAltoCm         = constrain(v,5,50);
  else if (k=="margenHuecoCm")     P.margenHuecoCm     = constrain(v,0,30);
  else if (k=="margenBanquetaCm")  P.margenBanquetaCm  = constrain(v,0,20);
  else if (k=="cmPorSegPark")      P.cmPorSegPark      = constrain(v,5,200);
  else if (k=="distInicialCm")     P.distInicialCm     = constrain(v,20,300);
  else if (k=="kickStartPWM")      P.kickStartPWM      = constrain(v,100,255);
  else if (k=="kickStartMs")       P.kickStartMs       = constrain(v,0,500);
  else if (k=="tReversaGiroMs")    P.tReversaGiroMs    = constrain(v,200,5000);
  else if (k=="lineFollowSpeed")   P.lineFollowSpeed   = constrain(v,0,255);
}

void handleCommand(String cmd, const String& sourceName) {
  cmd.trim();
  cmd.toUpperCase(); 
  if (cmd.length() == 0) return;

  lastAnyCmdMs = millis();
  publishLine("RX," + sourceName + "," + cmd);

  if (cmd.startsWith("SET:")) {
    int eq = cmd.indexOf('=');
    if (eq > 4) {
      String k = cmd.substring(4, eq); int v = cmd.substring(eq + 1).toInt();
      applyParam(k, v); publishLine("INFO,SET_OK," + k + "," + String(v));
    }
    return;
  }

  if (cmd.equals("PING")) { publishLine("PONG"); return; }
  
  bool isMovement = (cmd == "W" || cmd == "S" || cmd == "A" || cmd == "D" || cmd == "C");

  // SECCION DE BLINDAJE: Si el ESP32 está haciendo algo autónomo, ignoramos los comandos de movimiento 
  // que sigan llegando de la Raspberry Pi (UART) para que la IA no nos cancele el Seguidor de Linea
  if ((mode == AUTO_PARK || mode == TEST_PARK || mode == LINE_FOLLOW) && sourceName == "UART" && isMovement) {
    return; 
  }

  // Cancelacion manual desde GUI (TCP) o paros de emergencia
  if (mode == AUTO_PARK && cmd != "PR" && cmd != "PL") abortParking("manual_override");
  if (mode == TEST_PARK && cmd != "TPARKR" && cmd != "TPARKL") { publishLine("INFO,TPARK_CANCELLED"); fullStop(); mode = MANUAL; parkState = ABORTADO; }
  if (mode == LINE_FOLLOW && (isMovement || cmd == "X" || cmd == "ESTOP" || cmd == "STOP")) {
    stopLineFollow("manual_override");
  }

  // Comandos para elegir el modo de la Inteligencia Artificial (hacia la Pi)
  if      (cmd == "IA_ONLY") { PiSerial.println("IA_ONLY"); publishLine("INFO,MODO_IA_SOLO"); return; }
  else if (cmd == "IA_LF")   { PiSerial.println("IA_LF"); publishLine("INFO,MODO_IA_Y_LF"); return; }
  
  // Comandos para probar los focos del freno
  if      (cmd == "BRAKE_ON")  { stopLedOn(); publishLine("INFO,LUCES_FRENO_ENCENDIDAS"); return; }
  else if (cmd == "BRAKE_OFF") { stopLedOff(); publishLine("INFO,LUCES_FRENO_APAGADAS"); return; }

  if      (cmd == "STOP")   { mode = MANUAL; driveStop(); publishLine("INFO,STOP_SEÑAL_RECIBIDO"); }
  else if (cmd == "PR")     { if (mode == MANUAL) startParking(+1); }
  else if (cmd == "PL")     { if (mode == MANUAL) startParking(-1); }
  else if (cmd == "TPARKR") { startTestPark(+1); }
  else if (cmd == "TPARKL") { startTestPark(-1); }
  else if (cmd == "LF")     { startLineFollow(); }
  else if (cmd == "NOLF")   { if (mode == LINE_FOLLOW) stopLineFollow("nolf_cmd"); }
  else if (cmd == "T")      { mode = (mode == SENSOR_TEST) ? MANUAL : SENSOR_TEST; }
  else if (cmd == "H")      { hazardMode = (hazardMode == 0) ? 1 : 0; }
  else if (cmd == "HAZON")  { hazardMode = 1; }
  else if (cmd == "HAZOFF") { hazardMode = 0; }
  else if (cmd == "CAL")    { steerCalibrateHome(); steerCenter(); }
  else if (cmd == "ESTOP")  { abortParking("estop"); fullStop(); }
  else if (cmd == "W")      { mode = MANUAL; driveForward(); }
  else if (cmd == "S")      { mode = MANUAL; driveReverse(); }
  else if (cmd == "X")      { mode = MANUAL; driveStop(); }
  else if (cmd == "A")      { mode = MANUAL; steerRawLeft(); }
  else if (cmd == "D")      { mode = MANUAL; steerRawRight(); }
  else if (cmd == "C")      { mode = MANUAL; steerRawStop(); }
}

void acceptNewClients() {
  if (tcpServer.hasClient()) {
    WiFiClient newClient = tcpServer.available();
    int freeSlot = -1;
    for (int i = 0; i < 4; i++) { if (!tcpClients[i] || !tcpClients[i].connected()) { freeSlot = i; break; } }
    if (freeSlot >= 0) {
      if (tcpClients[freeSlot]) tcpClients[freeSlot].stop();
      tcpClients[freeSlot] = newClient; tcpBuffers[freeSlot] = "";
    } else newClient.stop();
  }
}

void cleanupClients() {
  for (int i = 0; i < 4; i++) { if (tcpClients[i] && !tcpClients[i].connected()) { tcpClients[i].stop(); tcpBuffers[i] = ""; } }
}

void readTcpCommands() {
  for (int i = 0; i < 4; i++) {
    if (tcpClients[i] && tcpClients[i].connected()) {
      while (tcpClients[i].available()) {
        char c = (char)tcpClients[i].read();
        if (c == '\n') { handleCommand(tcpBuffers[i], "TCP"); tcpBuffers[i] = ""; } 
        else if (c != '\r') { tcpBuffers[i] += c; if (tcpBuffers[i].length() > 200) tcpBuffers[i] = ""; }
      }
    }
  }
}

void readUARTCommands() {
  while (PiSerial.available()) {
    char c = (char)PiSerial.read();
    if (c == '\n') { handleCommand(uartBuffer, "UART"); uartBuffer = ""; } 
    else if (c != '\r') { uartBuffer += c; if (uartBuffer.length() > 200) uartBuffer = ""; }
  }
}

void handleGlobalTimeout() {
  if (mode == AUTO_PARK || mode == LINE_FOLLOW || mode == TEST_PARK) return;
  if (millis() - lastAnyCmdMs > GLOBAL_CMD_TIMEOUT_MS) fullStop();
}

void setup() {
  Serial.begin(115200);
  PiSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT); pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT); pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(LINE_L, INPUT); pinMode(LINE_R, INPUT); pinMode(LINE_C, INPUT);

  pinMode(LED_STOP, OUTPUT);
  digitalWrite(LED_STOP, LOW);

  ledcSetup(CH_STEER, PWM_FREQ, PWM_RES); ledcSetup(CH_DRIVE, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_STEER); ledcAttachPin(ENB, CH_DRIVE);

  fullStop(); delay(400); steerCalibrateHome(); steerCenter();

  WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID, AP_PASS);
  tcpServer.begin(); tcpServer.setNoDelay(true);
  xTaskCreatePinnedToCore(hazardTask, "hazard", 2048, NULL, 1, NULL, 0);
  lastAnyCmdMs = millis();
}

void loop() {
  steerUpdate(); leerSensores(); leerLineas();
  acceptNewClients(); cleanupClients(); readTcpCommands(); readUARTCommands();
  parkingLoop(); lineFollowLoop(); testParkLoop(); handleGlobalTimeout();

  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) { lastTelemetryMs = millis(); sendTelemetry(); }
  delay(10);
}