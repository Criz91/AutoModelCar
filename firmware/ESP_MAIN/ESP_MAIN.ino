// AutoModelCar - Firmware unificado ESP32-S3
// TMR 2026 - Categoria AutoModelCar
// Tres canales de comunicacion conviven:
//   - WiFi AP + TCP puerto 8080  -> GUI de la laptop (teleop, tuning, parking)
//   - UART2 (RX=16, TX=17)       -> Raspberry Pi 5 con Hailo (IA carril)
//   - Serial USB                  -> debug
//
// Modos:
//   MANUAL       - control manual WASD desde GUI o desde Pi
//   AUTO_PARK    - maquina de estados de estacionamiento en bateria
//   LINE_FOLLOW  - sigue linea con TCRT5000, activado por la Pi cuando
//                  Hailo pierde el carril en una curva cerrada
//   SENSOR_TEST  - dump de sensores cada 500 ms para depuracion

#include <WiFi.h>
#include <ArduinoJson.h>

// WIFI AP + TCP SERVER
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
// Direccion (motor delantero)
const int IN1 = 12, IN2 = 11, ENA = 8;

// Traccion (motor trasero)
const int IN3 = 14, IN4 = 13, ENB = 18;

// Ultrasonicos HC-SR04
const int TRIG_R = 4,  ECHO_R = 5;   // derecho
const int TRIG_L = 6,  ECHO_L = 7;   // izquierdo
const int TRIG_B = 15, ECHO_B = 9;   // trasero
const int TRIG_F = 1,  ECHO_F = 2;   // frontal (NUEVO)


// Seguidores de linea TCRT5000 (digitales, salida del comparador)
const int LINE_L = 38;   // izquierda
const int LINE_R = 39;   // derecha
const int LINE_C = 40;   // centro
// Polaridad: el modulo MDU-SEN-MH-TCRT5000 entrega LOW cuando ve superficie
// reflectiva (linea blanca sobre piso negro). Si la pista
// invierte la polaridad, cambiar LINE_ACTIVE_LOW a 0.
#define LINE_ACTIVE_LOW 1

// LEDs (intermitentes / hazards)
const int LED_L = 10, LED_R = 21;

// PWM
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int CH_STEER = 0;
const int CH_DRIVE = 1;

// PARAMETROS AJUSTABLES (sliders del GUI)
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

  // NUEVOS para LINE_FOLLOW
  int lineFollowSpeed    = 150;  // PWM de traccion durante LF
};
Params P;

// MODOS
enum Mode { MANUAL, AUTO_PARK, SENSOR_TEST, LINE_FOLLOW };
volatile Mode mode = MANUAL;

// ESTADOS DE ESTACIONADO
enum ParkState {
  IDLE,
  AVANCE_INICIAL,
  MEDIR_HUECO,
  AVANZAR_PASA_HUECO,
  STOP_Y_GIRAR,
  REVERSA_GIRO,
  STOP_Y_ENDEREZAR,
  REVERSA_RECTA,
  ESTACIONADO,
  ABORTADO
};

ParkState parkState = IDLE;
unsigned long stateStart = 0;
unsigned long autoStart  = 0;

int parkSide = +1;  // +1 derecha, -1 izquierda

// ESTIMACION DE DIRECCION
int steerPosMs   = 200;
int steerMoveDir = 0;
int steerMoveDur = 0;
unsigned long steerMoveStart = 0;

// HAZARDS
volatile int hazardMode = 0;

// FILTRO MEDIANA (5 muestras)
struct MedianFilter {
  long buf[5] = {999,999,999,999,999};
  int idx = 0;

  void push(long v) {
    buf[idx] = v;
    idx = (idx + 1) % 5;
  }

  long get() {
    long s[5];
    for (int i = 0; i < 5; i++) s[i] = buf[i];
    for (int i = 0; i < 4; i++) {
      for (int j = i + 1; j < 5; j++) {
        if (s[i] > s[j]) {
          long t = s[i];
          s[i] = s[j];
          s[j] = t;
        }
      }
    }
    return s[2];
  }
};

MedianFilter filtR, filtL, filtB, filtF;
volatile long lastR = 999, lastL = 999, lastB = 999, lastF = 999;

// Lecturas digitales de los seguidores de linea (1 = ve linea blanca)
volatile int lnL = 0, lnR = 0, lnC = 0;

// VARIABLES AUTO PARK
int countCarro = 0;
int countHueco = 0;
unsigned long huecoT0 = 0;
unsigned long huecoLastT = 0;
int huecoLargoCm = 0;
unsigned long kickStartT0 = 0;
bool kickStartActive = false;
bool kickArmPending = false;

// VARIABLES LINE FOLLOW
int  lfLastSteerTarget = -1;  // -1 = sin objetivo previo, evita refire
unsigned long lfStart = 0;

// TELEMETRIA Y TIEMPOS
unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 200;

unsigned long lastAnyCmdMs = 0;
const unsigned long GLOBAL_CMD_TIMEOUT_MS = 1500;

// UTILIDADES DE COMUNICACION
void sendToAllTcpClients(const String& msg) {
  for (int i = 0; i < 4; i++) {
    if (tcpClients[i] && tcpClients[i].connected()) {
      tcpClients[i].println(msg);
    }
  }
}

void publishLine(const String& msg) {
  Serial.println(msg);
  PiSerial.println(msg);
  sendToAllTcpClients(msg);
}

String modeToString() {
  switch (mode) {
    case MANUAL:
      if (parkState == ESTACIONADO) return "DONE";
      if (parkState == ABORTADO) return "ABORT";
      return "MANUAL";
    case AUTO_PARK:   return "AUTO";
    case SENSOR_TEST: return "TEST";
    case LINE_FOLLOW: return "LF";
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
  j["R"]    = lastR;
  j["L"]    = lastL;
  j["B"]    = lastB;
  j["F"]    = lastF;
  j["lnL"]  = lnL;
  j["lnR"]  = lnR;
  j["lnC"]  = lnC;
  j["sp"]   = steerPosMs;
  j["side"] = parkSide;

  char buf[384];
  size_t n = serializeJson(j, buf);

  Serial.println(buf);
  PiSerial.println(buf);
  for (int i = 0; i < 4; i++) {
    if (tcpClients[i] && tcpClients[i].connected()) {
      tcpClients[i].write((const uint8_t*)buf, n);
      tcpClients[i].write('\n');
    }
  }
}

// CONTROL DE MOTORES
void setMotor(int in1, int in2, int ch, int speed) {
  int s = constrain(abs(speed), 0, 255);

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

void driveForward(int s = -1) { setMotor(IN3, IN4, CH_DRIVE, +(s < 0 ? P.driveSpeed : s)); }
void driveReverse(int s = -1) { setMotor(IN3, IN4, CH_DRIVE, -(s < 0 ? P.driveSpeed : s)); }
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

bool steerBusy() {
  return steerMoveDir != 0;
}

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

void fullStop() {
  driveStop();
  steerRawStop();
}

// KICK START (vencer friccion estatica del motor parado)
void armKickStart() {
  kickStartT0 = millis();
  kickStartActive = true;
}

void driveForwardKick(int cruise) {
  int s;
  if (kickStartActive && (millis() - kickStartT0) < (unsigned long)P.kickStartMs) {
    s = P.kickStartPWM;
  } else {
    kickStartActive = false;
    s = cruise;
  }
  setMotor(IN3, IN4, CH_DRIVE, +s);
}

void driveReverseKick(int cruise) {
  int s;
  if (kickStartActive && (millis() - kickStartT0) < (unsigned long)P.kickStartMs) {
    s = P.kickStartPWM;
  } else {
    kickStartActive = false;
    s = cruise;
  }
  setMotor(IN3, IN4, CH_DRIVE, -s);
}

// SENSORES ULTRASONICOS
long medirCm(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH, 25000);
  if (dur == 0) return 999;
  return dur * 0.0343 / 2;
}

void pushSiValido(MedianFilter& f, long v) {
  if (v >= 5 && v < 999) f.push(v);
}

void leerSensores() {
  pushSiValido(filtR, medirCm(TRIG_R, ECHO_R));
  pushSiValido(filtL, medirCm(TRIG_L, ECHO_L));
  pushSiValido(filtB, medirCm(TRIG_B, ECHO_B));
  pushSiValido(filtF, medirCm(TRIG_F, ECHO_F));

  lastR = filtR.get();
  lastL = filtL.get();
  lastB = filtB.get();
  lastF = filtF.get();
}

// SEGUIDORES DE LINEA TCRT5000
inline int readLineSensor(int pin) {
  int raw = digitalRead(pin);
#if LINE_ACTIVE_LOW
  return (raw == LOW) ? 1 : 0;   // LOW = ve linea blanca
#else
  return (raw == HIGH) ? 1 : 0;
#endif
}

void leerLineas() {
  lnL = readLineSensor(LINE_L);
  lnR = readLineSensor(LINE_R);
  lnC = readLineSensor(LINE_C);
}

// HAZARD TASK (intermitentes en core 0)
void hazardTask(void *p) {
  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);

  bool on = false;
  int subStep = 0;

  while (true) {
    int delayMs = 300;

    switch (hazardMode) {
      case 0:
        digitalWrite(LED_L, LOW);
        digitalWrite(LED_R, LOW);
        delayMs = 100;
        break;

      case 1:
        on = !on;
        digitalWrite(LED_L, on);
        digitalWrite(LED_R, on);
        delayMs = 350;
        break;

      case 2:
        on = !on;
        digitalWrite(LED_L, on);
        digitalWrite(LED_R, on);
        delayMs = 120;
        break;

      case 3:
        digitalWrite(LED_L, HIGH);
        digitalWrite(LED_R, HIGH);
        delayMs = 200;
        break;

      case 4:
        subStep = (subStep + 1) % 6;
        if (subStep == 0 || subStep == 2) {
          digitalWrite(LED_L, HIGH);
          digitalWrite(LED_R, HIGH);
        } else {
          digitalWrite(LED_L, LOW);
          digitalWrite(LED_R, LOW);
        }
        delayMs = 120;
        break;
    }

    vTaskDelay(delayMs / portTICK_PERIOD_MS);
  }
}

// AUTO PARK - maquina de estados
void abortParking(const char* m) {
  publishLine(String("ABORT,") + m);
  fullStop();
  hazardMode = 4;
  mode = MANUAL;
  parkState = ABORTADO;
}

void finishParking() {
  publishLine("INFO,ESTACIONADO");
  fullStop();
  hazardMode = 3;
  mode = MANUAL;
  parkState = ESTACIONADO;
}

int tAvanceInicialMsEff() {
  if (P.tAvanceInicialMs > 0) return P.tAvanceInicialMs;
  if (P.cmPorSegPark <= 0) return 2500;
  return P.distInicialCm * 1000 / P.cmPorSegPark;
}

int tAvanzarMedioMsEff() {
  if (P.tAvanzarHuecoMs > 0) return P.tAvanzarHuecoMs;
  if (P.cmPorSegPark <= 0) return 700;
  int cm = P.carLargoCm / 2 + 4;
  return cm * 1000 / P.cmPorSegPark;
}

void startParking(int side) {
  parkSide = side;
  publishLine(String("INFO,AUTO_PARK_START,") + (side > 0 ? "RIGHT" : "LEFT"));

  mode = AUTO_PARK;
  hazardMode = 2;
  parkState = AVANCE_INICIAL;
  stateStart = millis();
  autoStart  = millis();

  countCarro = 0;
  countHueco = 0;
  huecoT0 = 0;
  huecoLastT = 0;
  huecoLargoCm = 0;

  steerCenter();
  kickArmPending = true;
}

void parkingLoop() {
  if (mode != AUTO_PARK) return;

  if (millis() - autoStart > (unsigned long)P.maxAutoMs) {
    abortParking("watchdog");
    return;
  }

  long dLatPark  = (parkSide > 0) ? lastR : lastL;
  long dLatCalle = (parkSide > 0) ? lastL : lastR;
  long dBack     = lastB;
  int huecoMinCm = P.carAnchoCm + P.margenHuecoCm;

  switch (parkState) {
    case AVANCE_INICIAL: {
      if (steerBusy()) {
        driveStop();
        break;
      }

      if (kickArmPending) {
        armKickStart();
        kickArmPending = false;
      }

      driveForwardKick(P.parkDriveSpeed);

      if (millis() - stateStart > (unsigned long)tAvanceInicialMsEff()) {
        publishLine("INFO,FIN_AVANCE_INICIAL");
        parkState = MEDIR_HUECO;
        stateStart = millis();
        huecoT0 = 0;
        huecoLastT = 0;
        huecoLargoCm = 0;
        countHueco = 0;
        countCarro = 0;
        hazardMode = 0;
      }
      break;
    }

    case MEDIR_HUECO: {
      driveForwardKick(P.parkDriveSpeed);

      bool enHueco = (dLatPark > P.distHuecoCm);
      bool enCarro = (dLatPark < P.distCarroCm);

      if (enHueco) {
        unsigned long now = millis();

        if (huecoT0 == 0) {
          huecoT0 = now;
          huecoLastT = now;
          huecoLargoCm = 0;
          countHueco = 1;
        } else {
          unsigned long dt = now - huecoLastT;
          huecoLastT = now;
          huecoLargoCm += (int)(dt * (unsigned long)P.cmPorSegPark / 1000UL);
          countHueco++;
        }

        if (huecoLargoCm >= huecoMinCm && countHueco >= 3) {
          publishLine(String("INFO,HUECO_OK,") + huecoLargoCm);
          hazardMode = 1;
          parkState = AVANZAR_PASA_HUECO;
          stateStart = millis();
          armKickStart();
          break;
        }
      } else if (enCarro) {
        if (huecoT0 != 0) {
          publishLine(String("INFO,HUECO_DESCARTADO,") + huecoLargoCm);
        }
        huecoT0 = 0;
        huecoLastT = 0;
        huecoLargoCm = 0;
        countHueco = 0;
      }

      if (millis() - stateStart > 20000) {
        abortParking("timeout_medir_hueco");
      }
      break;
    }

    case AVANZAR_PASA_HUECO: {
      driveForwardKick(P.parkDriveSpeed);
      if (millis() - stateStart > (unsigned long)tAvanzarMedioMsEff()) {
        driveStop();
        if (parkSide > 0) steerFullRight();
        else steerFullLeft();

        parkState = STOP_Y_GIRAR;
        stateStart = millis();
      }
      break;
    }

    case STOP_Y_GIRAR: {
      driveStop();
      if (!steerBusy()) {
        publishLine("INFO,DIRECCION_GIRADA");
        parkState = REVERSA_GIRO;
        stateStart = millis();
        armKickStart();
      }
      break;
    }

    case REVERSA_GIRO: {
      if (dLatPark < 4)  { abortParking("lateral_park_muy_cerca"); break; }
      if (dLatCalle < 6) { abortParking("lateral_calle_muy_cerca"); break; }

      driveReverseKick(P.parkDriveSpeed);

      if (dBack < P.distFrenaSuaveCm && dBack >= 5) {
        driveStop();
        publishLine("INFO,REVERSA_GIRO_FIN_SUAVE");
        parkState = STOP_Y_ENDEREZAR;
        stateStart = millis();
        steerCenter();
        break;
      }

      if (millis() - stateStart > (unsigned long)P.tReversaGiroMs) {
        driveStop();
        publishLine("INFO,REVERSA_GIRO_TIMEOUT");
        parkState = STOP_Y_ENDEREZAR;
        stateStart = millis();
        steerCenter();
      }
      break;
    }

    case STOP_Y_ENDEREZAR: {
      driveStop();
      if (!steerBusy()) {
        publishLine("INFO,DIRECCION_CENTRADA");
        parkState = REVERSA_RECTA;
        stateStart = millis();
        armKickStart();
      }
      break;
    }

    case REVERSA_RECTA: {
      if (dLatPark < 4)  { abortParking("lateral_park_muy_cerca"); break; }
      if (dLatCalle < 6) { abortParking("lateral_calle_muy_cerca"); break; }

      driveReverseKick((int)(P.parkDriveSpeed * 0.7));

      int dParo = P.distParaYaCm + P.margenBanquetaCm;
      if (dBack >= 5 && dBack < dParo) {
        finishParking();
        break;
      }

      if (millis() - stateStart > 4000) {
        publishLine("INFO,REVERSA_RECTA_TIMEOUT");
        finishParking();
      }
      break;
    }

    default:
      break;
  }
}

// LINE FOLLOW - corrige direccion con TCRT5000
// La Pi entra a este modo cuando Hailo pierde el carril en una curva
// cerrada (manda "LF") y sale cuando lo recupera (manda "NOLF"). El bucle
// de control vive en el ESP a 100 Hz para que no haya delay de USB+Pi.
//
// Convencion lnX = 1 significa "este sensor ve la linea blanca".
// Tabla de decision (centro, izquierdo, derecho):
//
//   C=1 L=0 R=0  ->  recto         (steerCenter)
//   C=1 L=1 R=0  ->  ajuste izq leve
//   C=1 L=0 R=1  ->  ajuste der leve
//   C=0 L=1 R=0  ->  girar izq fuerte
//   C=0 L=0 R=1  ->  girar der fuerte
//   C=0 L=1 R=1  ->  cruce / interseccion -> mantener
//   C=0 L=0 R=0  ->  perdio linea -> mantener (muy lento)
//   C=1 L=1 R=1  ->  parche / mancha -> mantener
//
// Velocidad de crucero menor que la manual para tener tiempo de reaccion.
void startLineFollow() {
  if (mode == AUTO_PARK) {
    publishLine("ERR,LF_BLOCKED_BY_AUTO_PARK");
    return;
  }
  publishLine("INFO,LF_START");
  mode = LINE_FOLLOW;
  hazardMode = 1;          // intermitente lento, se ve que esta en automatico
  lfStart = millis();
  lfLastSteerTarget = -1;
  armKickStart();
}

void stopLineFollow(const char* reason) {
  publishLine(String("INFO,LF_STOP,") + reason);
  driveStop();
  hazardMode = 0;
  mode = MANUAL;
  lfLastSteerTarget = -1;
}

void lineFollowLoop() {
  if (mode != LINE_FOLLOW) return;

  // Tiempo limite de seguridad por si la Pi se cuelga sin mandar NOLF
  if (millis() - lfStart > 15000UL) {
    stopLineFollow("watchdog_15s");
    return;
  }

  // Avance constante con kick start al inicio
  driveForwardKick(P.lineFollowSpeed);

  int center  = P.tSteerFullMs / 2 + P.steerTrimMs;
  int leftFull  = 0;
  int rightFull = P.tSteerFullMs;
  int leftSoft  = (int)(P.tSteerFullMs * 0.35);
  int rightSoft = (int)(P.tSteerFullMs * 0.65);

  int target = lfLastSteerTarget;   // por defecto mantener

  if      ( lnC && !lnL && !lnR) target = center;
  else if ( lnC &&  lnL && !lnR) target = leftSoft;
  else if ( lnC && !lnL &&  lnR) target = rightSoft;
  else if (!lnC &&  lnL && !lnR) target = leftFull;
  else if (!lnC && !lnL &&  lnR) target = rightFull;
  // C=0 L=1 R=1 (cruce), C=0 L=0 R=0 (perdido), C=1 L=1 R=1 (parche): mantener

  if (target != lfLastSteerTarget && target >= 0 && !steerBusy()) {
    steerGoTo(target);
    lfLastSteerTarget = target;
  }
}

// PARAMETROS POR COMANDO (SET:clave=valor)
void applyParam(const String& k, int v) {
  if      (k=="driveSpeed")        P.driveSpeed        = constrain(v,0,255);
  else if (k=="parkDriveSpeed")    P.parkDriveSpeed    = constrain(v,0,255);
  else if (k=="steerSpeed")        P.steerSpeed        = constrain(v,0,255);
  else if (k=="tSteerFullMs")      P.tSteerFullMs      = constrain(v,100,1500);
  else if (k=="steerTrimMs")       P.steerTrimMs       = constrain(v,-150,150);
  else if (k=="distCarroCm") {
    P.distCarroCm = constrain(v,5,100);
    if (P.distHuecoCm < P.distCarroCm + 10) P.distHuecoCm = P.distCarroCm + 10;
  }
  else if (k=="distHuecoCm") {
    P.distHuecoCm = constrain(v,10,200);
    if (P.distHuecoCm < P.distCarroCm + 10) P.distHuecoCm = P.distCarroCm + 10;
  }
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

// PARSER DE COMANDOS
// Comandos soportados (TCP, UART y Serial USB):
//   W S X A D C            - movimiento manual
//   PR PL                  - estacionar derecha / izquierda
//   LF NOLF                - entrar / salir de line follow
//   T H CAL ESTOP          - test sensores, hazards, calibrar, paro
//   SET:clave=valor        - ajustar parametro en vivo
//   PING                   - heartbeat (responde PONG)
void handleCommand(String cmd, const String& sourceName) {
  cmd.trim();
  if (cmd.length() == 0) return;

  lastAnyCmdMs = millis();
  publishLine("RX," + sourceName + "," + cmd);

  if (cmd.startsWith("SET:")) {
    int eq = cmd.indexOf('=');
    if (eq > 4) {
      String k = cmd.substring(4, eq);
      int v = cmd.substring(eq + 1).toInt();
      applyParam(k, v);
      publishLine("INFO,SET_OK," + k + "," + String(v));
    } else {
      publishLine("ERR,BAD_SET_FORMAT");
    }
    return;
  }

  if (cmd.equalsIgnoreCase("PING")) {
    publishLine("PONG");
    return;
  }

  // Cualquier comando manual durante AUTO_PARK lo cancela
  if (mode == AUTO_PARK && cmd != "PR" && cmd != "PL") {
    abortParking("manual_override");
  }

  // Cualquier comando manual de movimiento durante LF tambien lo cancela
  if (mode == LINE_FOLLOW &&
      (cmd == "W" || cmd == "S" || cmd == "X" ||
       cmd == "A" || cmd == "D" || cmd == "C" ||
       cmd == "ESTOP")) {
    stopLineFollow("manual_override");
  }

  if      (cmd == "PR")    { if (mode == MANUAL) startParking(+1); }
  else if (cmd == "PL")    { if (mode == MANUAL) startParking(-1); }
  else if (cmd == "LF")    { startLineFollow(); }
  else if (cmd == "NOLF")  { if (mode == LINE_FOLLOW) stopLineFollow("nolf_cmd"); }
  else if (cmd == "T")     { mode = (mode == SENSOR_TEST) ? MANUAL : SENSOR_TEST; publishLine("INFO,MODE," + modeToString()); }
  else if (cmd == "H")     { hazardMode = (hazardMode == 0) ? 1 : 0; publishLine("INFO,HAZARD," + String(hazardMode)); }
  else if (cmd == "CAL")   { steerCalibrateHome(); steerCenter(); publishLine("INFO,CAL_OK"); }
  else if (cmd == "ESTOP") { abortParking("estop"); fullStop(); }
  else if (cmd == "W")     { mode = MANUAL; driveForward(); }
  else if (cmd == "S")     { mode = MANUAL; driveReverse(); }
  else if (cmd == "X")     { mode = MANUAL; driveStop(); }
  else if (cmd == "A")     { mode = MANUAL; steerRawLeft(); }
  else if (cmd == "D")     { mode = MANUAL; steerRawRight(); }
  else if (cmd == "C")     { mode = MANUAL; steerRawStop(); }
  else {
    publishLine("ERR,UNKNOWN_COMMAND");
  }
}

// TCP - servidor para hasta 4 clientes
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
      if (tcpClients[freeSlot]) tcpClients[freeSlot].stop();

      tcpClients[freeSlot] = newClient;
      tcpBuffers[freeSlot] = "";

      tcpClients[freeSlot].println("INFO,TCP_CONNECTED");
      publishLine("INFO,TCP_CLIENT_CONNECTED," + String(freeSlot));
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
            publishLine("ERR,TCP_BUFFER_OVERFLOW");
          }
        }
      }
    }
  }
}

// UART - lee comandos de la Raspberry Pi
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
        publishLine("ERR,UART_BUFFER_OVERFLOW");
      }
    }
  }
}

// TIMEOUT DE SEGURIDAD GLOBAL
// Si no llega ningun comando en 1.5 s, frena. NO se aplica durante
// AUTO_PARK ni durante LINE_FOLLOW (esos modos son autonomos y no
// dependen de comandos externos continuos).
void handleGlobalTimeout() {
  if (mode == AUTO_PARK)   return;
  if (mode == LINE_FOLLOW) return;

  unsigned long now = millis();
  if (now - lastAnyCmdMs > GLOBAL_CMD_TIMEOUT_MS) {
    driveStop();
    steerRawStop();
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("BOOTING...");

  PiSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);

  pinMode(LINE_L, INPUT);
  pinMode(LINE_R, INPUT);
  pinMode(LINE_C, INPUT);

  ledcSetup(CH_STEER, PWM_FREQ, PWM_RES);
  ledcSetup(CH_DRIVE, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_STEER);
  ledcAttachPin(ENB, CH_DRIVE);

  fullStop();
  delay(400);
  steerCalibrateHome();
  steerCenter();

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

  xTaskCreatePinnedToCore(hazardTask, "hazard", 2048, NULL, 1, NULL, 0);

  lastAnyCmdMs = millis();

  publishLine("INFO,SYSTEM_READY");
  publishLine("INFO,TCP_PORT,8080");
  publishLine("INFO,UART_READY,115200");
  publishLine("INFO,MODE,MANUAL");
}

// LOOP PRINCIPAL (~100 Hz)
void loop() {
  steerUpdate();
  leerSensores();
  leerLineas();

  acceptNewClients();
  cleanupClients();
  readTcpCommands();
  readUARTCommands();

  parkingLoop();
  lineFollowLoop();
  handleGlobalTimeout();

  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();
    sendTelemetry();
  }

  static unsigned long tPrint = 0;
  if (mode == SENSOR_TEST && millis() - tPrint > 500) {
    tPrint = millis();
    Serial.printf("R:%ld  L:%ld  B:%ld  F:%ld  ln:%d%d%d\n",
                  lastR, lastL, lastB, lastF, lnL, lnC, lnR);
    PiSerial.printf("R:%ld  L:%ld  B:%ld  F:%ld  ln:%d%d%d\n",
                    lastR, lastL, lastB, lastF, lnL, lnC, lnR);
  }

  delay(10);
}
