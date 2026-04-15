#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Firmware del carro AutoModelCar para la prueba de estacionamiento
// autonomo en bateria del TMR 2026. Se conecta por WiFi en modo AP y
// expone un WebSocket por el cual el GUI de Python manda comandos
// manuales (WASD), arranca la maniobra automatica (PR/PL) y ajusta
// parametros en vivo (SET:clave=valor).

// Credenciales del Access Point que crea el ESP32. El GUI se conecta
// a esta red WiFi y abre el WebSocket en 192.168.4.1/ws.
const char* ap_ssid = "ESP32_PRUEBA";
const char* ap_pass = "123456785";

// Pines del puente H. IN1/IN2/ENA controlan el motor de direccion
// (llantas delanteras). IN3/IN4/ENB controlan el motor de traccion
// (llantas traseras). ENA y ENB reciben la senal PWM de velocidad.
// Importante: estos numeros deben coincidir con el cableado fisico,
// porque el LEDC attach al pin ENA/ENB decide a que pin sale el PWM.
const int IN1 = 12, IN2 = 11, ENA = 8;    // motor de direccion
const int IN3 = 14, IN4 = 13, ENB = 18;   // motor de traccion

// Pines de los tres ultrasonicos HC-SR04. R = derecho, L = izquierdo,
// B = trasero. Cada sensor usa un pin TRIG (salida) y uno ECHO (entrada).
const int TRIG_R = 4,  ECHO_R = 5;
const int TRIG_L = 6,  ECHO_L = 7;
const int TRIG_B = 15, ECHO_B = 9;

// Pines de los dos LEDs que hacen de intermitentes/hazards.
const int LED_L = 10, LED_R = 21;

// Configuracion de PWM por LEDC. 20 kHz queda fuera del rango audible
// (no se oye chillido del motor) y 8 bits da un duty de 0 a 255.
// Se usan dos canales independientes, uno por motor.
const int PWM_FREQ = 20000, PWM_RES = 8;
const int CH_STEER = 0, CH_DRIVE = 1;

// Parametros que el GUI puede modificar en vivo via "SET:clave=valor".
// Cualquier ajuste de calibracion o tuning pasa por aqui sin recompilar.
struct Params {
  int driveSpeed         = 180;   // PWM traccion manual
  int parkDriveSpeed     = 200;   // PWM traccion auto (suficiente para vencer friccion estatica)
  int steerSpeed         = 170;   // PWM direccion (mas suave)
  int tSteerFullMs       = 400;   // tope a tope de la direccion
  int steerTrimMs        = 0;     // sesgo de centro (-150..+150)
  int distCarroCm        = 25;    // ve carro si <
  int distHuecoCm        = 45;    // ve hueco si >
  int distFrenaSuaveCm   = 15;    // frenar suave atras
  int distParaYaCm       = 8;     // parar definitivamente atras
  int tAvanceInicialMs   = 0;     // override manual (0 = derivar de distInicialCm/cmPorSegPark)
  int tAvanzarHuecoMs    = 0;     // override manual (0 = derivar de carLargoCm/cmPorSegPark)
  int minHuecoStableMs   = 250;   // hueco debe sostenerse este tiempo (override manual)
  int maxAutoMs          = 60000; // watchdog del modo auto

  // Geometria fisica del carro y calibracion de la maniobra.
  // El equipo midio el chasis con regla: 29 x 14 x 25 cm.
  int carLargoCm         = 29;    // largo del carro
  int carAnchoCm         = 14;    // ancho del carro
  int carAltoCm          = 25;    // alto (informativo)
  int margenHuecoCm      = 6;     // colchon de seguridad lateral
  int margenBanquetaCm   = 4;     // colchon de seguridad trasero
  int cmPorSegPark       = 30;    // velocidad real estimada a parkDriveSpeed (CALIBRAR)
  int distInicialCm      = 100;   // 1 m del reglamento
  int kickStartPWM       = 240;   // PWM de arranque para vencer friccion estatica
  int kickStartMs        = 150;   // duracion del kick-start
  int tReversaGiroMs     = 1600;  // duracion de reversa con giro al tope
};
Params P;

// Modo de operacion global. MANUAL responde a WASD del GUI,
// AUTO_PARK ejecuta la maniobra de estacionamiento y SENSOR_TEST
// imprime las lecturas de los HC-SR04 por Serial para depurar.
enum Mode { MANUAL, AUTO_PARK, SENSOR_TEST };
volatile Mode mode = MANUAL;

// Estados de la maquina de estados de estacionamiento. El flujo normal
// es: AVANCE_INICIAL (recorre 1 m) -> MEDIR_HUECO (busca un hueco
// suficientemente largo con el sensor lateral) -> AVANZAR_PASA_HUECO
// (avanza hasta que el eje trasero quede en el borde del hueco) ->
// STOP_Y_GIRAR (gira la direccion al tope) -> REVERSA_GIRO (entra en
// diagonal) -> STOP_Y_ENDEREZAR (centra direccion) -> REVERSA_RECTA
// (acomoda hasta antes de la banqueta) -> ESTACIONADO. ABORTADO se
// activa por watchdog, override manual o por sensores en peligro.
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

int parkSide = +1;  // +1=derecha, -1=izquierda

// Posicion estimada de la direccion. El motor de direccion no es un
// servo: es un motor DC con puente H, asi que no sabemos su posicion
// real. Aqui se estima cronometrando el tiempo que lleva moviendose
// en cada sentido a steerSpeed PWM. tSteerFullMs es el tiempo de tope
// a tope. Por eso al inicio steerCalibrateHome choca contra el tope
// izquierdo para fijar el cero.
int steerPosMs   = 200;
int steerMoveDir = 0;
int steerMoveDur = 0;
unsigned long steerMoveStart = 0;

// Modo actual de los LEDs (intermitentes y luces de estado).
// 0=apagados, 1=intermitente lento, 2=intermitente rapido,
// 3=fijos (auto OK), 4=doble flash (abort/error).
volatile int hazardMode = 0;

// Filtro de mediana de 5 muestras por sensor. Se usa para tirar
// outliers de un solo frame (un eco raro de la pista, una rebote
// del piso) sin retrasar mucho la respuesta del sensor.
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
unsigned long huecoT0 = 0;        // inicio de hueco abierto
unsigned long huecoLastT = 0;     // ultimo frame valido del hueco actual
int huecoLargoCm = 0;             // longitud integrada del hueco actual
unsigned long kickStartT0 = 0;    // inicio del kick-start del estado actual
bool kickStartActive = false;     // true mientras se aplica PWM alto
bool kickArmPending = false;      // pendiente de armar kick-start cuando termine la direccion

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Helper de bajo nivel para mover un motor por puente H. Cada motor
// tiene dos pines de direccion (in1, in2) y un canal PWM (ch). El
// signo de speed decide el sentido de giro y el valor absoluto el
// duty (0..255). Speed = 0 deja el motor libre (coast).
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

// Arma un kick-start: marca el instante para que las siguientes llamadas
// a driveForwardKick / driveReverseKick usen kickStartPWM por kickStartMs ms.
void armKickStart() {
  kickStartT0 = millis();
  kickStartActive = true;
}

// Avanza con kick-start: PWM alto al inicio para vencer friccion estatica.
// Despues del periodo cae a cruise (parkDriveSpeed por defecto).
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

// Lectura puntual de un HC-SR04. Manda un pulso de 10 us en TRIG,
// mide cuanto tarda en volver el eco por ECHO y lo convierte a cm
// con la velocidad del sonido (343 m/s). Si no hay eco antes de
// 25 ms (rango > ~4 m o sin reflejo) devuelve 999 como "sin lectura".
long medirCm(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 25000);
  if (dur == 0) return 999;
  return dur * 0.0343 / 2;
}

// Empuja al filtro solo si la lectura es valida (no 999 ni <5).
// Si es invalida, no actualiza el filtro (mantiene la mediana anterior).
void pushSiValido(MedianFilter& f, long v) {
  if (v >= 5 && v < 999) f.push(v);
}

void leerSensores() {
  pushSiValido(filtR, medirCm(TRIG_R, ECHO_R));
  pushSiValido(filtL, medirCm(TRIG_L, ECHO_L));
  pushSiValido(filtB, medirCm(TRIG_B, ECHO_B));
  lastR = filtR.get();
  lastL = filtL.get();
  lastB = filtB.get();
}

// Tarea FreeRTOS dedicada al parpadeo de los LEDs intermitentes.
// Vive en el core 0 para no competir con el loop principal y solo
// lee la variable global hazardMode para decidir el patron actual.
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

// Arranca la maniobra automatica. side = +1 estaciona a la derecha,
// side = -1 a la izquierda. Hace un preflash bloqueante de 5 destellos
// como aviso visual antes de empezar a moverse y arma el kick-start
// del primer estado para vencer la friccion estatica del tren motriz.
void startParking(int side) {
  parkSide = side;
  Serial.print(">>> AUTO PARK START side=");
  Serial.println(side > 0 ? "DERECHA" : "IZQUIERDA");

  // No bloqueamos el handler del WebSocket. La hazardTask hara el
  // patron rapido durante el avance inicial como aviso visual y se
  // apagara cuando empiece a buscar el hueco.
  mode = AUTO_PARK;
  hazardMode = 2;  // intermitente rapido durante avance inicial
  parkState = AVANCE_INICIAL;
  stateStart = millis();
  autoStart  = millis();
  countCarro = 0;
  countHueco = 0;
  huecoT0 = 0;
  huecoLastT = 0;
  huecoLargoCm = 0;
  steerCenter();
  // No armamos el kick-start aqui: la direccion esta centrandose y
  // los 150 ms se gastarian en vano. AVANCE_INICIAL lo arma cuando
  // detecta que la direccion ya termino.
  kickArmPending = true;
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

// Helper: avance inicial en cm efectivo (override manual o derivado)
int tAvanceInicialMsEff() {
  if (P.tAvanceInicialMs > 0) return P.tAvanceInicialMs;
  if (P.cmPorSegPark <= 0)    return 2500;
  return P.distInicialCm * 1000 / P.cmPorSegPark;
}
// Helper: tiempo para avanzar carLargoCm/2 + margen tras detectar fin de hueco
int tAvanzarMedioMsEff() {
  if (P.tAvanzarHuecoMs > 0) return P.tAvanzarHuecoMs;
  if (P.cmPorSegPark <= 0)   return 700;
  int cm = P.carLargoCm / 2 + 4;
  return cm * 1000 / P.cmPorSegPark;
}

void parkingLoop() {
  if (mode != AUTO_PARK) return;

  // Watchdog global
  if (millis() - autoStart > (unsigned long)P.maxAutoMs) {
    abortParking("watchdog");
    return;
  }

  long dLatPark = (parkSide > 0) ? lastR : lastL;  // sensor del lado del estacionamiento
  long dLatCalle = (parkSide > 0) ? lastL : lastR; // sensor del lado opuesto
  long dBack = lastB;
  int  huecoMinCm = P.carAnchoCm + P.margenHuecoCm;

  switch (parkState) {

    case AVANCE_INICIAL: {
      // Avanza recto el "1 metro inicial" sin buscar nada
      if (steerBusy()) { driveStop(); break; }
      // La direccion ya termino. Si veniamos de startParking con el
      // kick-start pendiente, lo armamos justo ahora para que los 150 ms
      // de PWM alto se apliquen en el primer movimiento real.
      if (kickArmPending) {
        armKickStart();
        kickArmPending = false;
      }
      driveForwardKick(P.parkDriveSpeed);
      if (millis() - stateStart > (unsigned long)tAvanceInicialMsEff()) {
        Serial.println("Fin avance inicial -> medir hueco");
        // Pasar directo a MEDIR_HUECO. La logica de MEDIR_HUECO maneja
        // los dos casos: el primer cajon esta vacio (empieza a integrar
        // de inmediato), o esta ocupado (espera con la rama enCarro hasta
        // que termine y empiece el hueco real).
        parkState = MEDIR_HUECO;
        stateStart = millis();
        huecoT0 = 0;
        huecoLastT = 0;
        huecoLargoCm = 0;
        countHueco = 0;
        countCarro = 0;
        // Apagamos hazards: durante la busqueda van apagadas y se
        // prenden cuando confirmemos el hueco.
        hazardMode = 0;
      }
      break;
    }

    case MEDIR_HUECO: {
      driveForwardKick(P.parkDriveSpeed);

      // Estamos en hueco si dLatPark > distHuecoCm. Si esta entre los
      // dos umbrales (banda muerta) no contamos ese frame ni a favor
      // ni en contra: mantiene el estado actual sin modificar la integral.
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
          // Integral incremental: solo suma el dt entre dos frames
          // que ambos vieron hueco. Si pasamos por banda muerta, esos
          // ms no se cuentan (mas conservador, no sobrestima el hueco).
          unsigned long dt = now - huecoLastT;
          huecoLastT = now;
          huecoLargoCm += (int)(dt * (unsigned long)P.cmPorSegPark / 1000UL);
          countHueco++;
        }
        if (huecoLargoCm >= huecoMinCm && countHueco >= 3) {
          Serial.print("Hueco OK, largo="); Serial.println(huecoLargoCm);
          hazardMode = 1; // intermitentes ON al confirmar el hueco
          parkState = AVANZAR_PASA_HUECO;
          stateStart = millis();
          armKickStart();
          break;
        }
      } else if (enCarro) {
        // Hueco se cerro antes de alcanzar el largo minimo -> descartar
        if (huecoT0 != 0) {
          Serial.print("Hueco descartado, largo="); Serial.println(huecoLargoCm);
        }
        huecoT0 = 0;
        huecoLastT = 0;
        huecoLargoCm = 0;
        countHueco = 0;
      }
      // Banda muerta: no toca huecoT0/huecoLargoCm/countHueco. La
      // integral se reanuda en el siguiente frame que vea hueco real.

      if (millis() - stateStart > 20000) abortParking("timeout medir hueco");
      break;
    }

    case AVANZAR_PASA_HUECO: {
      driveForwardKick(P.parkDriveSpeed);
      if (millis() - stateStart > (unsigned long)tAvanzarMedioMsEff()) {
        driveStop();
        if (parkSide > 0) steerFullRight(); else steerFullLeft();
        parkState = STOP_Y_GIRAR;
        stateStart = millis();
      }
      break;
    }

    case STOP_Y_GIRAR: {
      driveStop();
      if (!steerBusy()) {
        Serial.println("Direccion girada -> reversa con giro");
        parkState = REVERSA_GIRO;
        stateStart = millis();
        armKickStart();
      }
      break;
    }

    case REVERSA_GIRO: {
      // Seguridad: si algun lateral se acerca peligrosamente, abortar
      if (dLatPark < 4)  { abortParking("lateral park muy cerca"); break; }
      if (dLatCalle < 6) { abortParking("lateral calle muy cerca"); break; }

      driveReverseKick(P.parkDriveSpeed);

      // Termina por: tiempo, freno suave trasero, o parada total trasera
      if (dBack < P.distFrenaSuaveCm && dBack >= 5) {
        driveStop();
        Serial.println("Reversa giro -> enderezar (freno suave)");
        parkState = STOP_Y_ENDEREZAR;
        stateStart = millis();
        steerCenter();
        break;
      }
      if (millis() - stateStart > (unsigned long)P.tReversaGiroMs) {
        driveStop();
        Serial.println("Reversa giro -> enderezar (timeout)");
        parkState = STOP_Y_ENDEREZAR;
        stateStart = millis();
        steerCenter();
      }
      break;
    }

    case STOP_Y_ENDEREZAR: {
      driveStop();
      if (!steerBusy()) {
        Serial.println("Direccion centrada -> reversa recta");
        parkState = REVERSA_RECTA;
        stateStart = millis();
        armKickStart();
      }
      break;
    }

    case REVERSA_RECTA: {
      // Seguridad lateral durante la reversa final
      if (dLatPark < 4)  { abortParking("lateral park muy cerca"); break; }
      if (dLatCalle < 6) { abortParking("lateral calle muy cerca"); break; }

      // Avance lento (70% del cruise)
      driveReverseKick((int)(P.parkDriveSpeed * 0.7));

      // Detenerse antes de tocar la banqueta
      int dParo = P.distParaYaCm + P.margenBanquetaCm;
      if (dBack >= 5 && dBack < dParo) {
        Serial.println("REVERSA RECTA -> FIN");
        finishParking();
        break;
      }
      if (millis() - stateStart > 4000) {
        Serial.println("REVERSA RECTA timeout -> FIN");
        finishParking();
      }
      break;
    }

    default: break;
  }
}

// Devuelve el nombre del modo actual para mandar al GUI. Cuando ya
// terminamos la maniobra (estado ESTACIONADO o ABORTADO) reportamos
// DONE / ABORT en lugar de MANUAL para que el GUI lo pinte distinto.
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

// Aplica un parametro recibido por WebSocket ("SET:clave=valor").
// Cada clave conocida se valida con constrain a un rango razonable
// para que un valor erroneo del GUI no rompa la maniobra.
void applyParam(const String& k, int v) {
  if      (k=="driveSpeed")        P.driveSpeed        = constrain(v,0,255);
  else if (k=="parkDriveSpeed")    P.parkDriveSpeed    = constrain(v,0,255);
  else if (k=="steerSpeed")        P.steerSpeed        = constrain(v,0,255);
  else if (k=="tSteerFullMs")      P.tSteerFullMs      = constrain(v,100,1500);
  else if (k=="steerTrimMs")       P.steerTrimMs       = constrain(v,-150,150);
  else if (k=="distCarroCm") {
    P.distCarroCm = constrain(v,5,100);
    // distHuecoCm siempre debe estar al menos 10 cm por encima de
    // distCarroCm para tener una banda muerta util y evitar oscilaciones.
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
  // Parametros de geometria y calibracion del modo automatico.
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

// Inicializa pines, PWM, calibracion de la direccion, levanta el AP
// WiFi y arranca la tarea de hazards. Despues de esto el carro queda
// en modo MANUAL esperando comandos por WebSocket.
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