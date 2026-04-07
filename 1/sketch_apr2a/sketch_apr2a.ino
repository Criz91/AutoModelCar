// Archivo: sketch_apr2a.ino
// Sistema hibrido: Control manual por WebSocket y Estacionamiento Autonomo
// Placa: ESP32-S3 (Version 2.x)

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Configuracion de Red WiFi
const char* ap_ssid = "ESP32_CAR1";
const char* ap_pass = "12345678";

// Pines para el Puente H (Motor de Direccion)
const int IN1 = 12;
const int IN2 = 13;
const int ENA = 18; 

// Pines para el Puente H (Motor de Traccion)
const int IN3 = 14;
const int IN4 = 11;
const int ENB = 19; 

// Pines para Sensores Ultrasonicos
#define TRIG_FRENTE   4
#define ECHO_FRENTE   5
#define TRIG_LATERAL  6
#define ECHO_LATERAL  7
#define TRIG_ATRAS    15
#define ECHO_ATRAS    16
#define TRIG_ESQUINA  17
#define ECHO_ESQUINA  21 

// Pines para LEDs intermitentes
#define LED_DERECHO   10
#define LED_IZQUIERDO 2  

// Configuracion PWM para motores
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int CH_STEER = 0;
const int CH_DRIVE = 1;

// Velocidades de operacion
int speedManual = 180; 
int speedAuto   = 120; 
int steerSpeed  = 180; 

// Servidor WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Variables globales de control
volatile bool modoAutonomo = false;
volatile bool intermitentesActivas = false;
TaskHandle_t taskBlink = NULL;

// Variables y medidas fisicas del entorno
float ANCHO_CARRO    = 25.5f;
float LARGO_CARRO    = 35.0f;
float MARGEN         = 6.0f;
float ESPACIO_MINIMO = 31.5f; 
float DIST_CARRO     = 30.0f; 
float DIST_BANQUETA  = 5.0f;

// Tiempos limite de seguridad en milisegundos
#define TIEMPO_MAX_REVERSA  15000UL
#define TIEMPO_MAX_HUECO     8000UL

// Variables de estado del estacionamiento
unsigned long tiempoInicioHueco   = 0;
unsigned long tiempoInicioReversa = 0;
float espacioDetectado = 0.0f;
float velocidadCarroCmS = 15.0f; 

// Maquina de estados
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

// Tarea en segundo plano para las luces intermitentes (FreeRTOS)
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
      digitalWrite(LED_DERECHO,   LOW);
      digitalWrite(LED_IZQUIERDO, LOW);
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}

// Funciones de control de motores (Version 2.x)
void setMotor(int in1, int in2, int ch, int speed) {
  int s = constrain(abs(speed), 0, 255);
  if      (speed > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  ledcWrite(ch, s);
}

void steerLeft()       { setMotor(IN1, IN2, CH_STEER, -steerSpeed); }
void steerRight()      { setMotor(IN1, IN2, CH_STEER, +steerSpeed); }
void steerCenter()     { setMotor(IN1, IN2, CH_STEER,  0); }
void driveStop()       { setMotor(IN3, IN4, CH_DRIVE,  0); }
void fullStop()        { driveStop(); steerCenter(); }

// Funcion para leer distancia del sensor ultrasonico
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

// Manejo de mensajes de control remoto por WebSocket
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) return;
  if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->opcode != WS_TEXT) return;

    String cmd;
    for (size_t i = 0; i < len; i++) cmd += (char)data[i];

    if (cmd == "W" || cmd == "S" || cmd == "X" || cmd == "A" || cmd == "D" || cmd == "C") {
      if (modoAutonomo) {
        modoAutonomo = false;
        intermitentesActivas = false;
        Serial.println("OVERRIDE: Regresando a modo manual");
      }
      
      if      (cmd == "W") setMotor(IN3, IN4, CH_DRIVE, speedManual);
      else if (cmd == "S") setMotor(IN3, IN4, CH_DRIVE, -speedManual);
      else if (cmd == "X") fullStop();
      else if (cmd == "A") steerLeft();
      else if (cmd == "D") steerRight();
      else if (cmd == "C") steerCenter();
    }
    else if (cmd == "P") {
      estadoActual         = AVANCE_INICIAL;
      estadoAnterior       = AVANCE_INICIAL;
      espacioDetectado     = 0.0f;
      intermitentesActivas = false;
      modoAutonomo         = true;
      Serial.println("Iniciando estacionamiento autonomo");
    }
  }

  if (type == WS_EVT_DISCONNECT) {
    modoAutonomo = false;
    intermitentesActivas = false;
    fullStop();
  }
}

// Funcion clave: Espera sin bloquear el codigo
bool esperarConSeguridad(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    if (!modoAutonomo) {
      fullStop(); 
      return false; 
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
  return true; 
}

// Logica principal de la maquina de estados para estacionamiento
void runParking() {
  float frente  = medirDistancia(TRIG_FRENTE,  ECHO_FRENTE);
  float lateral = medirDistancia(TRIG_LATERAL, ECHO_LATERAL);
  float atras   = medirDistancia(TRIG_ATRAS,   ECHO_ATRAS);
  float esquina = medirDistancia(TRIG_ESQUINA, ECHO_ESQUINA);

  if (estadoActual != estadoAnterior) {
    Serial.print("ESTADO: "); Serial.println(estadoActual);
    estadoAnterior = estadoActual;
  }

  switch (estadoActual) {

    case AVANCE_INICIAL: {
      int msAvance = (int)((100.0f / velocidadCarroCmS) * 1000.0f);
      setMotor(IN3, IN4, CH_DRIVE, speedAuto); 
      
      if (!esperarConSeguridad(msAvance)) return; 
      
      driveStop();

      if (lateral >= DIST_CARRO) {
        tiempoInicioHueco = millis();
        estadoActual = CALCULANDO;
      } else {
        estadoActual = AVANZANDO;
      }
      break;
    }

    case AVANZANDO:
      setMotor(IN3, IN4, CH_DRIVE, speedAuto);
      if (!esperarConSeguridad(100)) return;
      
      if (lateral < DIST_CARRO) {
        estadoActual = BUSCANDO_HUECO;
      }
      break;

    case BUSCANDO_HUECO:
      setMotor(IN3, IN4, CH_DRIVE, speedAuto);
      if (!esperarConSeguridad(100)) return;
      
      if (lateral >= DIST_CARRO) {
        tiempoInicioHueco = millis();
        estadoActual = CALCULANDO;
      }
      break;

    case CALCULANDO: {
      setMotor(IN3, IN4, CH_DRIVE, speedAuto);
      unsigned long tiempoHueco = millis() - tiempoInicioHueco;
      if (!esperarConSeguridad(100)) return;

      if (lateral < DIST_CARRO) {
        driveStop();
        espacioDetectado = (tiempoHueco / 1000.0f) * velocidadCarroCmS;

        if (espacioDetectado >= ESPACIO_MINIMO) {
          estadoActual = POSICIONANDO;
        } else {
          estadoActual = BUSCANDO_HUECO;
        }
      }
      else if (tiempoHueco > TIEMPO_MAX_HUECO) {
        driveStop();
        espacioDetectado = (tiempoHueco / 1000.0f) * velocidadCarroCmS;
        estadoActual = POSICIONANDO;
      }
      break;
    }

    case POSICIONANDO: {
      intermitentesActivas = true; 

      float avanzar = (espacioDetectado / 2.0f) - (ANCHO_CARRO / 2.0f);
      avanzar = max(avanzar, 0.0f); 

      setMotor(IN3, IN4, CH_DRIVE, speedAuto);
      if (!esperarConSeguridad((int)((avanzar / velocidadCarroCmS) * 1000.0f))) return;
      
      driveStop();
      if (!esperarConSeguridad(500)) return;

      steerLeft();
      if (!esperarConSeguridad(800)) return;
      
      steerCenter();
      if (!esperarConSeguridad(300)) return;

      estadoActual = ESPERANDO;
      break;
    }

    case ESPERANDO:
      if (!esperarConSeguridad(400)) return;

      if (frente > 40.0f) {
        if (!esperarConSeguridad(2000)) return;
        
        float frenteVerif = medirDistancia(TRIG_FRENTE, ECHO_FRENTE);
        if (frenteVerif > 40.0f) {
          tiempoInicioReversa = millis();
          estadoActual = REVERSANDO;
        }
      }
      break;

    case REVERSANDO: {
      unsigned long tiempoReversando = millis() - tiempoInicioReversa;

      if (tiempoReversando > TIEMPO_MAX_REVERSA) {
        driveStop();
        estadoActual = ERROR_PARADA;
        break;
      }

      if (esquina < 8.0f) steerLeft();
      else                steerCenter();

      setMotor(IN3, IN4, CH_DRIVE, -speedAuto);
      if (!esperarConSeguridad(50)) return;

      if (atras <= DIST_BANQUETA) {
        driveStop();
        steerCenter();
        estadoActual = ESTACIONADO;
      }
      break;
    }

    case ESTACIONADO:
      driveStop();
      steerCenter();
      intermitentesActivas = false;
      modoAutonomo = false;
      Serial.println("ESTACIONADO CORRECTAMENTE");
      break;

    case ERROR_PARADA:
      driveStop();
      steerCenter();
      intermitentesActivas = false;
      modoAutonomo = false;
      Serial.println("ERROR DE SEGURIDAD");
      break;
  }
}

// Inicializacion general
void setup() {
  Serial.begin(115200);
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcSetup(CH_STEER, PWM_FREQ, PWM_RES);
  ledcSetup(CH_DRIVE, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_STEER);
  ledcAttachPin(ENB, CH_DRIVE);

  pinMode(TRIG_FRENTE,  OUTPUT); pinMode(ECHO_FRENTE,  INPUT);
  pinMode(TRIG_LATERAL, OUTPUT); pinMode(ECHO_LATERAL, INPUT);
  pinMode(TRIG_ATRAS,   OUTPUT); pinMode(ECHO_ATRAS,   INPUT);
  pinMode(TRIG_ESQUINA, OUTPUT); pinMode(ECHO_ESQUINA, INPUT);

  pinMode(LED_DERECHO,   OUTPUT);
  pinMode(LED_IZQUIERDO, OUTPUT);

  fullStop();

  xTaskCreatePinnedToCore(taskIntermitentesFunc, "blink", 1024, NULL, 1, &taskBlink, 0);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  
  Serial.println("Sistema encendido y esperando conexion");
}

// Bucle principal
void loop() {
  if (modoAutonomo) {
    runParking();
  }
}