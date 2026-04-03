// ================================================================
//  CarroTMR1 — sketch_jan26a.ino
//  Control manual (WebSocket) + Estacionamiento autónomo
//
//  MODOS:
//    MANUAL   → comandos WASD/XC por WebSocket (default)
//    AUTÓNOMO → secuencia de estacionamiento (comando "P")
//
//  OVERRIDE DE SEGURIDAD:
//    Cualquier tecla W/S/A/D/X/C durante modo autónomo
//    regresa INMEDIATAMENTE a modo manual y frena.
//
//  INTERMITENTES:
//    Se activan al entrar a POSICIONANDO y parpadean
//    continuamente hasta ESTACIONADO (tarea FreeRTOS).
// ================================================================

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ── Red WiFi (Access Point) ──────────────────────────────────
const char* ap_ssid = "ESP32_CAR1";
const char* ap_pass = "12345678";

// ── Pines: Puente H — Motor de DIRECCIÓN ────────────────────
const int IN1 = 12;
const int IN2 = 13;
const int ENA = 18;   // PWM dirección

// ── Pines: Puente H — Motor de TRACCIÓN ─────────────────────
const int IN3 = 14;
const int IN4 = 11;
const int ENB = 19;   // PWM tracción

// ── Pines: Sensores ultrasónicos ─────────────────────────────
//   NOTA: ECHO_ESQUINA cambió de 18→21 (18 ya es ENA)
#define TRIG_FRENTE   4
#define ECHO_FRENTE   5
#define TRIG_LATERAL  6
#define ECHO_LATERAL  7
#define TRIG_ATRAS    15
#define ECHO_ATRAS    16
#define TRIG_ESQUINA  17
#define ECHO_ESQUINA  21   // ← cambiado (era 18, conflicto con ENA)

// ── Pines: LEDs intermitentes ────────────────────────────────
//   NOTA: LED_IZQUIERDO cambió de 11→2 (11 ya es IN4)
#define LED_DERECHO   10
#define LED_IZQUIERDO 2    // ← cambiado (era 11, conflicto con IN4)

// ── PWM ──────────────────────────────────────────────────────
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int CH_STEER = 0;
const int CH_DRIVE = 1;

int driveSpeed = 180;
int steerSpeed = 160;

// ── Servidor WebSocket ────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ── Modo de operación ────────────────────────────────────────
volatile bool modoAutonomo = false;

// ── Intermitentes (controladas por tarea FreeRTOS) ───────────
volatile bool intermitentesActivas = false;
TaskHandle_t taskBlink = NULL;

// ── Variables del estacionamiento ────────────────────────────
float ANCHO_CARRO    = 25.5f;
float LARGO_CARRO    = 35.0f;
float MARGEN         = 6.0f;
float ESPACIO_MINIMO = 31.5f;   // ANCHO_CARRO + MARGEN
float DIST_CARRO     = 30.0f;
float DIST_BANQUETA  = 5.0f;
float velocidadCarro = 20.0f;

#define TIEMPO_MAX_REVERSA  15000UL
#define TIEMPO_MAX_HUECO     8000UL

unsigned long tiempoInicioHueco   = 0;
unsigned long tiempoInicioReversa = 0;
float espacioDetectado = 0.0f;

enum Estado {
  AVANCE_INICIAL,
  AVANZANDO,
  BUSCANDO_HUECO,
  CALCULANDO,
  POSICIONANDO,
  ESPERANDO,
  REVERSANDO,
  ESTACIONADO,
  ERROR_PARADA
};
volatile Estado estadoActual   = AVANCE_INICIAL;
volatile Estado estadoAnterior = AVANCE_INICIAL;

// ================================================================
//  TAREA FREERTOS — Intermitentes independientes
//  Parpadea a 300ms mientras intermitentesActivas == true.
//  Corre en Core 0 para no interferir con WiFi (Core 1).
// ================================================================
void taskIntermitentesFunc(void* pvParameters) {
  for (;;) {
    if (intermitentesActivas) {
      digitalWrite(LED_DERECHO,   HIGH);
      digitalWrite(LED_IZQUIERDO, HIGH);
      vTaskDelay(300 / portTICK_PERIOD_MS);
      digitalWrite(LED_DERECHO,   LOW);
      digitalWrite(LED_IZQUIERDO, LOW);
      vTaskDelay(300 / portTICK_PERIOD_MS);
    } else {
      // Asegurar apagados cuando no activas
      digitalWrite(LED_DERECHO,   LOW);
      digitalWrite(LED_IZQUIERDO, LOW);
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}

// ================================================================
//  FUNCIONES DE MOTOR
// ================================================================
void setMotor(int in1, int in2, int ch, int speed) {
  int s = constrain(abs(speed), 0, 255);
  if      (speed > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  ledcWrite(ch, s);
}

void steerLeft()    { setMotor(IN1, IN2, CH_STEER, -steerSpeed); }
void steerRight()   { setMotor(IN1, IN2, CH_STEER, +steerSpeed); }
void steerCenter()  { setMotor(IN1, IN2, CH_STEER,  0); }
void driveForward() { setMotor(IN3, IN4, CH_DRIVE, +driveSpeed); }
void driveReverse() { setMotor(IN3, IN4, CH_DRIVE, -driveSpeed); }
void driveStop()    { setMotor(IN3, IN4, CH_DRIVE,  0); }
void fullStop()     { driveStop(); steerCenter(); }

// ================================================================
//  SENSOR ULTRASÓNICO
// ================================================================
float medirDistancia(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  long dur = pulseIn(pinEcho, HIGH, 30000);
  if (dur == 0) return 999.0f;
  return dur * 0.034f / 2.0f;
}

// ================================================================
//  MANEJADOR WEBSOCKET
//  • WASD/X/C → modo manual inmediato (override de seguridad)
//  • "P"      → iniciar estacionamiento autónomo
// ================================================================
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) return;

  if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->opcode != WS_TEXT) return;

    String cmd;
    for (size_t i = 0; i < len; i++) cmd += (char)data[i];

    // ── Comandos de movimiento manual ────────────────────────
    // Si llega cualquiera de estos en modo autónomo → OVERRIDE
    if (cmd == "W" || cmd == "S" || cmd == "X" ||
        cmd == "A" || cmd == "D" || cmd == "C") {

      if (modoAutonomo) {
        modoAutonomo        = false;
        intermitentesActivas = false;
        Serial.println(">> [OVERRIDE] Operador tomó el control — MODO MANUAL");
      }

      if      (cmd == "W") driveForward();
      else if (cmd == "S") driveReverse();
      else if (cmd == "X") fullStop();
      else if (cmd == "A") steerLeft();
      else if (cmd == "D") steerRight();
      else if (cmd == "C") steerCenter();
    }

    // ── Comando "P" → iniciar estacionamiento autónomo ───────
    else if (cmd == "P") {
      estadoActual        = AVANCE_INICIAL;
      estadoAnterior      = AVANCE_INICIAL;
      espacioDetectado    = 0.0f;
      intermitentesActivas = false;
      modoAutonomo        = true;
      Serial.println(">> [AUTO] Iniciando estacionamiento autónomo...");
    }
  }

  // Al desconectarse el control → frena y regresa a manual
  if (type == WS_EVT_DISCONNECT) {
    modoAutonomo        = false;
    intermitentesActivas = false;
    fullStop();
  }
}

// ================================================================
//  DEBUG — imprimir nombre del estado
// ================================================================
void imprimirEstado(Estado e) {
  Serial.print(">> ESTADO: ");
  switch (e) {
    case AVANCE_INICIAL: Serial.println("AVANCE INICIAL"); break;
    case AVANZANDO:      Serial.println("AVANZANDO"); break;
    case BUSCANDO_HUECO: Serial.println("BUSCANDO HUECO"); break;
    case CALCULANDO:     Serial.println("CALCULANDO"); break;
    case POSICIONANDO:   Serial.println("POSICIONANDO"); break;
    case ESPERANDO:      Serial.println("ESPERANDO"); break;
    case REVERSANDO:     Serial.println("REVERSANDO"); break;
    case ESTACIONADO:    Serial.println("¡ESTACIONADO!"); break;
    case ERROR_PARADA:   Serial.println("ERROR — PARADA"); break;
  }
}

// Macro de seguridad: dentro de runParking(), después de cada delay largo
// verifica si el operador intervino. Si sí, sale de la función inmediatamente.
#define CHECK_OVERRIDE()  if (!modoAutonomo) { fullStop(); return; }

// ================================================================
//  MÁQUINA DE ESTADOS — ESTACIONAMIENTO AUTÓNOMO
//  Se llama desde loop() cuando modoAutonomo == true.
// ================================================================
void runParking() {
  float frente  = medirDistancia(TRIG_FRENTE,  ECHO_FRENTE);
  float lateral = medirDistancia(TRIG_LATERAL, ECHO_LATERAL);
  float atras   = medirDistancia(TRIG_ATRAS,   ECHO_ATRAS);
  float esquina = medirDistancia(TRIG_ESQUINA, ECHO_ESQUINA);

  // Imprimir cambio de estado
  if (estadoActual != estadoAnterior) {
    imprimirEstado(estadoActual);
    estadoAnterior = estadoActual;
  }

  switch (estadoActual) {

    // ── Avanzar ~1 metro antes de buscar hueco ──────────────
    case AVANCE_INICIAL: {
      int msAvance = (int)((100.0f / velocidadCarro) * 1000.0f);
      Serial.println("[AVANCE INICIAL] Avanzando 100 cm...");
      driveForward();
      delay(msAvance);
      CHECK_OVERRIDE();
      driveStop();

      // ¿Ya hay hueco desde el inicio?
      if (lateral >= DIST_CARRO) {
        tiempoInicioHueco = millis();
        estadoActual = CALCULANDO;
      } else {
        estadoActual = AVANZANDO;
      }
      break;
    }

    // ── Avanzar buscando el primer carro obstáculo ──────────
    case AVANZANDO:
      driveForward();
      delay(100);
      CHECK_OVERRIDE();
      if (lateral < DIST_CARRO) {
        Serial.println("[AVANZANDO] Primer carro detectado");
        estadoActual = BUSCANDO_HUECO;
      }
      break;

    // ── Esperar a que el lateral se abra (inicio del hueco) ─
    case BUSCANDO_HUECO:
      driveForward();
      delay(100);
      CHECK_OVERRIDE();
      if (lateral >= DIST_CARRO) {
        Serial.println("[BUSCANDO HUECO] Hueco detectado — midiendo...");
        tiempoInicioHueco = millis();
        estadoActual = CALCULANDO;
      }
      break;

    // ── Medir el ancho del hueco mientras se pasa ───────────
    case CALCULANDO: {
      driveForward();
      unsigned long tiempoHueco = millis() - tiempoInicioHueco;

      Serial.print("[CALCULANDO] lateral: "); Serial.print(lateral);
      Serial.print(" cm | tiempo: "); Serial.print(tiempoHueco / 1000.0f, 1);
      Serial.println(" s");
      delay(100);
      CHECK_OVERRIDE();

      // ¿Terminó el hueco? (vemos el segundo carro)
      if (lateral < DIST_CARRO) {
        driveStop();
        espacioDetectado = (tiempoHueco / 1000.0f) * velocidadCarro;
        Serial.print(">> Espacio: "); Serial.print(espacioDetectado);
        Serial.print(" cm | mínimo requerido: "); Serial.println(ESPACIO_MINIMO);

        if (espacioDetectado >= ESPACIO_MINIMO) {
          estadoActual = POSICIONANDO;
        } else {
          Serial.println(">> No cabe — buscando otro hueco");
          estadoActual = BUSCANDO_HUECO;
        }
      }
      // Timeout: hueco muy grande o no hay segundo carro
      else if (tiempoHueco > TIEMPO_MAX_HUECO) {
        driveStop();
        espacioDetectado = (tiempoHueco / 1000.0f) * velocidadCarro;
        Serial.println(">> Timeout — asumiendo que cabe");
        estadoActual = POSICIONANDO;
      }
      break;
    }

    // ── Centrarse en el hueco + activar intermitentes ───────
    case POSICIONANDO: {
      // ★ Activar intermitentes — seguirán hasta ESTACIONADO ★
      intermitentesActivas = true;

      float avanzar = (espacioDetectado / 2.0f) - (ANCHO_CARRO / 2.0f);
      avanzar = max(avanzar, 0.0f);   // No retroceder si el cálculo da negativo
      Serial.print("[POSICIONANDO] Avanzando "); Serial.print(avanzar); Serial.println(" cm");

      driveForward();
      delay((int)((avanzar / velocidadCarro) * 1000.0f));
      CHECK_OVERRIDE();
      driveStop();
      delay(500);
      CHECK_OVERRIDE();

      // Abrirse a la izquierda para la maniobra de reversa
      Serial.println("[POSICIONANDO] Abriéndose a la izquierda...");
      steerLeft();
      delay(800);
      CHECK_OVERRIDE();
      steerCenter();
      delay(300);
      CHECK_OVERRIDE();

      estadoActual = ESPERANDO;
      break;
    }

    // ── Esperar que el carril quede libre para reversar ─────
    case ESPERANDO:
      Serial.print("[ESPERANDO] frente: "); Serial.print(frente); Serial.println(" cm");
      delay(400);
      CHECK_OVERRIDE();

      if (frente > 40.0f) {
        Serial.println(">> Carril parece libre — verificando 2 s...");
        delay(2000);
        CHECK_OVERRIDE();
        float frenteVerif = medirDistancia(TRIG_FRENTE, ECHO_FRENTE);
        if (frenteVerif > 40.0f) {
          Serial.println(">> Carril LIBRE — iniciando reversa");
          tiempoInicioReversa = millis();
          estadoActual = REVERSANDO;
        } else {
          Serial.print(">> Algo apareció ("); Serial.print(frenteVerif); Serial.println(" cm) — esperando");
        }
      }
      break;

    // ── Reversar hacia el cajón ──────────────────────────────
    case REVERSANDO: {
      unsigned long tiempoReversando = millis() - tiempoInicioReversa;

      Serial.print("[REVERSANDO] atras: "); Serial.print(atras);
      Serial.print(" cm | esquina: "); Serial.print(esquina);
      Serial.print(" cm | t: "); Serial.print(tiempoReversando / 1000.0f, 1); Serial.println(" s");

      // Timeout de seguridad
      if (tiempoReversando > TIEMPO_MAX_REVERSA) {
        driveStop();
        Serial.println(">> TIMEOUT reversa — parada de seguridad");
        estadoActual = ERROR_PARADA;
        break;
      }

      // Corrección de esquina trasera derecha
      if (esquina < 8.0f) {
        Serial.print(">> ALERTA esquina: "); Serial.print(esquina); Serial.println(" cm — corrección");
        steerLeft();
      } else {
        steerCenter();
      }

      driveReverse();
      delay(50);
      CHECK_OVERRIDE();

      // ¿Llegó a la banqueta?
      if (atras <= DIST_BANQUETA) {
        driveStop();
        steerCenter();
        estadoActual = ESTACIONADO;
      }
      break;
    }

    // ── Estacionado correctamente ────────────────────────────
    case ESTACIONADO:
      driveStop();
      steerCenter();
      intermitentesActivas = false;          // ★ Apagar intermitentes ★
      digitalWrite(LED_DERECHO,  LOW);
      digitalWrite(LED_IZQUIERDO, LOW);
      modoAutonomo = false;
      Serial.println();
      Serial.println("====================================");
      Serial.println("     ¡ESTACIONADO CORRECTAMENTE!   ");
      Serial.println("====================================");
      delay(5000);
      break;

    // ── Error: parada de seguridad ───────────────────────────
    case ERROR_PARADA:
      driveStop();
      steerCenter();
      intermitentesActivas = false;
      modoAutonomo = false;
      Serial.println(">> ERROR activo — se requiere intervención manual");
      delay(2000);
      break;
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n== CarroTMR1 arrancando ==");

  // ── Pines motores
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // ── PWM
  ledcSetup(CH_STEER, PWM_FREQ, PWM_RES);
  ledcSetup(CH_DRIVE, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_STEER);
  ledcAttachPin(ENB, CH_DRIVE);

  // ── Pines sensores
  pinMode(TRIG_FRENTE,  OUTPUT); pinMode(ECHO_FRENTE,  INPUT);
  pinMode(TRIG_LATERAL, OUTPUT); pinMode(ECHO_LATERAL, INPUT);
  pinMode(TRIG_ATRAS,   OUTPUT); pinMode(ECHO_ATRAS,   INPUT);
  pinMode(TRIG_ESQUINA, OUTPUT); pinMode(ECHO_ESQUINA, INPUT);

  // ── LEDs
  pinMode(LED_DERECHO,   OUTPUT);
  pinMode(LED_IZQUIERDO, OUTPUT);

  // ── Freno inicial
  fullStop();

  // ── Tarea FreeRTOS para intermitentes (Core 0, prioridad 1)
  xTaskCreatePinnedToCore(
    taskIntermitentesFunc, "blink",
    1024, NULL, 1, &taskBlink, 0
  );

  // ── WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.print("AP listo — IP: ");
  Serial.println(WiFi.softAPIP());

  // ── WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  // ── Parpadeo de confirmación (3 veces)
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_DERECHO,   HIGH);
    digitalWrite(LED_IZQUIERDO, HIGH);
    delay(200);
    digitalWrite(LED_DERECHO,   LOW);
    digitalWrite(LED_IZQUIERDO, LOW);
    delay(200);
  }

  Serial.println("Sistema listo.");
  Serial.println("  WASD/XC = control manual");
  Serial.println("  P       = estacionamiento autónomo");
  Serial.println("  (cualquier WASD durante autónomo = override manual)");
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  if (modoAutonomo) {
    runParking();
  }
  // En modo manual: todo se maneja por callbacks de AsyncWebSocket
  // loop() queda libre — no bloquea el WiFi
}
