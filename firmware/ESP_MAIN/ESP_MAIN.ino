// AutoModelCar TMR 2026 - Prueba de estacionamiento autonomo en bateria
// Plataforma: ESP32-S3
// Direccion: motor DC de carrito (sin encoder), control por tiempo
// Traccion: motores DC amarillos via L298N

#include <WiFi.h>
#include <WiFiServer.h>

// Pines L298N - traccion
#define PIN_IN3 14
#define PIN_IN4 13
#define PIN_ENB 18

// Pines L298N - direccion
#define PIN_IN1 12
#define PIN_IN2 11
#define PIN_ENA 8

// Pines ultrasonicos
#define PIN_TRIG_DER 4
#define PIN_ECHO_DER 5
#define PIN_TRIG_IZQ 6
#define PIN_ECHO_IZQ 7
#define PIN_TRIG_TRA 15
#define PIN_ECHO_TRA 9
#define PIN_TRIG_FRE 1
#define PIN_ECHO_FRE 2

// Pines LEDs
#define PIN_LED_IZQ   10
#define PIN_LED_DER   21
#define PIN_LED_FRENO 41

// Canales PWM (API vieja ESP32 Arduino)
#define CH_TRACCION  1
#define CH_DIRECCION 0
#define PWM_FRECUENCIA 20000
#define PWM_BITS       8

// WiFi AP
const char* WIFI_SSID = "AutoModelCar";
const char* WIFI_PASS = "automodel123";
const int   TCP_PUERTO = 8080;

// Timeout modo MANUAL (ms)
const unsigned long TIMEOUT_MANUAL_MS = 1500;

// Intervalo telemetria (ms)
const unsigned long INTERVALO_TELEMETRIA_MS = 200;

// Intervalo lectura ultrasonicos - rotacion de un sensor por tick (ms)
const unsigned long INTERVALO_SENSOR_MS = 60;

// Longitud maxima del buffer de comandos
const int BUFFER_MAX = 64;

struct Parametros {
    // Velocidades (0-255)
    int velocidadAvance    = 150;
    int velocidadReversa   = 150;
    int velocidadParking   = 100;
    int velocidadDireccion = 200;

    // Direccion
    int tiempoDireccionTope = 400; // ms tope a tope

    // Kickstart (superar friccion estatica)
    int kickPwm = 240;
    int kickMs  = 150;

    // Tiempos Fase A - test ciego (ms)
    int tFrenoInicialMs  = 5000;
    int tAvance1Ms       = 2000;
    int tAvance2Ms       = 2000;
    int tGiroIzqMs       = 1000;
    int tReversaGiroMs   = 2000;
    int tReversaRectaMs  = 4000;

    // Fase B - sensores (cm)
    int distCarroCm      = 15;  // umbral "hay carro" (histeresis bajo)
    int distHuecoCm      = 25;  // umbral "hay hueco" (histeresis alto)
    int distBanquetaCm   = 5;   // parar antes de banqueta
    int distLateralMinCm = 3;   // lateral muy cerca -> maniobra correctiva
    int largoCarroCm     = 25;  // largo estimado del vehiculo
    int margenHuecoCm    = 5;   // margen extra para confirmar que cabe
    int cmPorSegundo     = 30;  // calibrar en Fase A (cm/s a velocidadParking)
    int muestrasEstables = 3;   // muestras consecutivas para confirmar lectura
};

enum Modo {
    MANUAL,
    TEST_CIEGO,
    ESTACIONAR_DER,
    ESTACIONADO,
    ABORTADO
};

enum EstadoRutina {
    NINGUNO,
    // Test ciego (Fase A)
    TC_INICIO,
    TC_AVANCE_1,
    TC_FRENO_1,
    TC_AVANCE_2,
    TC_FRENO_2,
    TC_GIRO_IZQ,
    TC_FRENO_3,
    TC_REVERSA_GIRO,
    TC_FRENO_4,
    TC_ENDEREZAR,
    TC_FRENO_5,
    TC_REVERSA_RECTA,
    TC_FIN
    // Fase B se agregara cuando Fase A este calibrada
};

enum ModoIntermitentes {
    INT_OFF,
    INT_IZQ,
    INT_DER,
    INT_AMBAS,
    INT_AMBAS_FIJAS
};

enum LadoDireccion {
    DIR_IZQ,
    DIR_DER,
    DIR_STOP
};

// Variables globales
Parametros P;
Modo         modoActual    = MANUAL;
EstadoRutina estadoRutina  = NINGUNO;
ModoIntermitentes modoInter = INT_OFF;

// Estado de traccion
int  pwmTraccionActual    = 0;
bool traccionAdelante     = true;

// Estado de direccion
LadoDireccion ladoDireccion     = DIR_STOP;
int           posicionDireccion = 0; // 0=tope izq, tiempoDireccionTope=tope der
unsigned long tInicioDireccion  = 0;

// Maquina de estados
bool          estadoIniciado    = false;
unsigned long tInicioEstado     = 0;
unsigned long durEnderezarMs    = 0; // tiempo calculado para centrar

// Intermitentes
unsigned long tUltimoBlink = 0;
bool          estadoBlink  = false;

// Timeout manual
unsigned long tUltimoComando = 0;

// Telemetria
unsigned long tUltimaTelemetria = 0;

// Ultrasonicos - lectura en rotacion (un sensor por tick)
float distDer = 999, distIzq = 999, distTra = 999, distFre = 999;
unsigned long tUltimoSensor = 0;
int sensorTurno = 0; // 0=DER 1=IZQ 2=TRA 3=FRE

// Medicion de hueco (Fase B)
float huecoMedidoCm = 0;

// TCP
WiFiServer servidor(TCP_PUERTO);
WiFiClient cliente;
String     bufferTCP  = "";

// UART2 (Raspberry Pi)
String bufferUART = "";


// ─── TRACCION ────────────────────────────────────────────────────────────────

void aplicarTraccion(int pwm, bool adelante) {
    pwmTraccionActual = pwm;
    traccionAdelante  = adelante;
    if (pwm == 0) {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        ledcWrite(CH_TRACCION, 0);
    } else {
        digitalWrite(PIN_IN3, adelante ? HIGH : LOW);
        digitalWrite(PIN_IN4, adelante ? LOW  : HIGH);
        ledcWrite(CH_TRACCION, pwm);
    }
}

void traccionAvanzar(int pwm) { aplicarTraccion(pwm, true);  }
void traccionReversa(int pwm) { aplicarTraccion(pwm, false); }
void traccionDetener()        { aplicarTraccion(0,   true);  }


// ─── DIRECCION ───────────────────────────────────────────────────────────────

// Actualiza posicionDireccion con el tiempo transcurrido desde el ultimo movimiento
void actualizarPosicion() {
    if (ladoDireccion == DIR_STOP) return;
    unsigned long ahora   = millis();
    unsigned long elapsed = ahora - tInicioDireccion;
    tInicioDireccion = ahora;
    if (ladoDireccion == DIR_IZQ) posicionDireccion -= (int)elapsed;
    else                          posicionDireccion += (int)elapsed;
    posicionDireccion = constrain(posicionDireccion, 0, P.tiempoDireccionTope);
}

void direccionIzquierda() {
    actualizarPosicion();
    ladoDireccion    = DIR_IZQ;
    tInicioDireccion = millis();
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(CH_DIRECCION, P.velocidadDireccion);
}

void direccionDerecha() {
    actualizarPosicion();
    ladoDireccion    = DIR_DER;
    tInicioDireccion = millis();
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    ledcWrite(CH_DIRECCION, P.velocidadDireccion);
}

void direccionDetener() {
    actualizarPosicion();
    ladoDireccion = DIR_STOP;
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(CH_DIRECCION, 0);
}

// Calibra el tope izquierdo al arrancar para tener origen conocido
void calibrarDireccion() {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(CH_DIRECCION, P.velocidadDireccion);
    delay((int)(P.tiempoDireccionTope * 1.3));
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(CH_DIRECCION, 0);
    posicionDireccion = 0;
    ladoDireccion     = DIR_STOP;
    tInicioDireccion  = millis();
}


// ─── ULTRASONICOS ────────────────────────────────────────────────────────────

// Mediana de 3 lecturas, timeout 15 ms (~250 cm maximo)
float medirCm(int trigPin, int echoPin) {
    float m[3];
    for (int i = 0; i < 3; i++) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        long dur = pulseIn(echoPin, HIGH, 15000);
        m[i] = dur == 0 ? 999.0f : dur * 0.017f;
        delayMicroseconds(300);
    }
    // Ordenar 3 elementos
    if (m[0] > m[1]) { float t = m[0]; m[0] = m[1]; m[1] = t; }
    if (m[1] > m[2]) { float t = m[1]; m[1] = m[2]; m[2] = t; }
    if (m[0] > m[1]) { float t = m[0]; m[0] = m[1]; m[1] = t; }
    return m[1];
}

// Lee un sensor por turno para no bloquear el loop
void leerSensorTurno() {
    switch (sensorTurno) {
        case 0: distDer = medirCm(PIN_TRIG_DER, PIN_ECHO_DER); break;
        case 1: distIzq = medirCm(PIN_TRIG_IZQ, PIN_ECHO_IZQ); break;
        case 2: distTra = medirCm(PIN_TRIG_TRA, PIN_ECHO_TRA); break;
        case 3: distFre = medirCm(PIN_TRIG_FRE, PIN_ECHO_FRE); break;
    }
    sensorTurno = (sensorTurno + 1) % 4;
}


// ─── LEDS ────────────────────────────────────────────────────────────────────

void actualizarLEDs() {
    // Luces de freno: encendidas cuando la traccion esta detenida
    digitalWrite(PIN_LED_FRENO, pwmTraccionActual == 0 ? HIGH : LOW);

    // Intermitentes
    unsigned long ahora = millis();
    switch (modoInter) {
        case INT_OFF:
            digitalWrite(PIN_LED_IZQ, LOW);
            digitalWrite(PIN_LED_DER, LOW);
            break;
        case INT_AMBAS_FIJAS:
            digitalWrite(PIN_LED_IZQ, HIGH);
            digitalWrite(PIN_LED_DER, HIGH);
            break;
        default:
            if (ahora - tUltimoBlink >= 500) {
                tUltimoBlink = ahora;
                estadoBlink  = !estadoBlink;
            }
            bool b = estadoBlink;
            digitalWrite(PIN_LED_IZQ, (modoInter == INT_IZQ || modoInter == INT_AMBAS) ? (b ? HIGH : LOW) : LOW);
            digitalWrite(PIN_LED_DER, (modoInter == INT_DER || modoInter == INT_AMBAS) ? (b ? HIGH : LOW) : LOW);
            break;
    }
}


// ─── TELEMETRIA ──────────────────────────────────────────────────────────────

const char* nombreModo() {
    switch (modoActual) {
        case MANUAL:         return "MANUAL";
        case TEST_CIEGO:     return "TEST_CIEGO";
        case ESTACIONAR_DER: return "ESTACIONAR_DER";
        case ESTACIONADO:    return "ESTACIONADO";
        case ABORTADO:       return "ABORTADO";
    }
    return "?";
}

const char* nombreEstado() {
    switch (estadoRutina) {
        case NINGUNO:          return "NINGUNO";
        case TC_INICIO:        return "TC_INICIO";
        case TC_AVANCE_1:      return "TC_AVANCE_1";
        case TC_FRENO_1:       return "TC_FRENO_1";
        case TC_AVANCE_2:      return "TC_AVANCE_2";
        case TC_FRENO_2:       return "TC_FRENO_2";
        case TC_GIRO_IZQ:      return "TC_GIRO_IZQ";
        case TC_FRENO_3:       return "TC_FRENO_3";
        case TC_REVERSA_GIRO:  return "TC_REVERSA_GIRO";
        case TC_FRENO_4:       return "TC_FRENO_4";
        case TC_ENDEREZAR:     return "TC_ENDEREZAR";
        case TC_FRENO_5:       return "TC_FRENO_5";
        case TC_REVERSA_RECTA: return "TC_REVERSA_RECTA";
        case TC_FIN:           return "TC_FIN";
    }
    return "?";
}

const char* nombreInter() {
    switch (modoInter) {
        case INT_OFF:         return "OFF";
        case INT_IZQ:         return "IZQ";
        case INT_DER:         return "DER";
        case INT_AMBAS:       return "AMBAS";
        case INT_AMBAS_FIJAS: return "FIJAS";
    }
    return "OFF";
}

void enviarTelemetria() {
    char buf[300];
    snprintf(buf, sizeof(buf),
        "{\"modo\":\"%s\",\"estado\":\"%s\",\"tEstado\":%lu,"
        "\"dR\":%.1f,\"dL\":%.1f,\"dB\":%.1f,\"dF\":%.1f,"
        "\"velTrac\":%d,\"posDir\":%d,\"inter\":\"%s\","
        "\"freno\":%s,\"hueco\":%.1f}\n",
        nombreModo(), nombreEstado(), millis() - tInicioEstado,
        distDer, distIzq, distTra, distFre,
        pwmTraccionActual, posicionDireccion, nombreInter(),
        pwmTraccionActual == 0 ? "true" : "false",
        huecoMedidoCm
    );
    if (cliente && cliente.connected()) cliente.print(buf);
    Serial.print(buf);
}


// ─── MAQUINA DE ESTADOS ──────────────────────────────────────────────────────

void cambiarEstado(EstadoRutina nuevo) {
    estadoRutina   = nuevo;
    tInicioEstado  = millis();
    estadoIniciado = false;
}

void pararTodo() {
    traccionDetener();
    direccionDetener();
    modoActual    = MANUAL;
    estadoRutina  = NINGUNO;
    modoInter     = INT_OFF;
    estadoIniciado = false;
}

void loopTestCiego() {
    unsigned long tEnEstado = millis() - tInicioEstado;

    switch (estadoRutina) {

        case TC_INICIO:
            // 5 s detenido simulando encendido (freno ON por regla general)
            if (tEnEstado >= (unsigned long)P.tFrenoInicialMs) {
                cambiarEstado(TC_AVANCE_1);
            }
            break;

        case TC_AVANCE_1:
            if (!estadoIniciado) {
                estadoIniciado = true;
                traccionAvanzar(P.velocidadParking);
            }
            if (tEnEstado >= (unsigned long)P.tAvance1Ms) {
                traccionDetener();
                cambiarEstado(TC_FRENO_1);
            }
            break;

        case TC_FRENO_1:
            if (tEnEstado >= 500) {
                cambiarEstado(TC_AVANCE_2);
            }
            break;

        case TC_AVANCE_2:
            if (!estadoIniciado) {
                estadoIniciado = true;
                traccionAvanzar(P.velocidadParking);
            }
            if (tEnEstado >= (unsigned long)P.tAvance2Ms) {
                traccionDetener();
                cambiarEstado(TC_FRENO_2);
            }
            break;

        case TC_FRENO_2:
            // Activa intermitentes al encontrar el hueco (simulado)
            if (!estadoIniciado) {
                estadoIniciado = true;
                modoInter = INT_AMBAS;
            }
            if (tEnEstado >= 500) {
                cambiarEstado(TC_GIRO_IZQ);
            }
            break;

        case TC_GIRO_IZQ:
            // Direccion izq + avanzar simultaneamente
            if (!estadoIniciado) {
                estadoIniciado = true;
                direccionIzquierda();
                traccionAvanzar(P.velocidadParking);
            }
            if (tEnEstado >= (unsigned long)P.tGiroIzqMs) {
                traccionDetener();
                direccionDetener();
                cambiarEstado(TC_FRENO_3);
            }
            break;

        case TC_FRENO_3:
            if (tEnEstado >= 500) {
                cambiarEstado(TC_REVERSA_GIRO);
            }
            break;

        case TC_REVERSA_GIRO:
            // Direccion der + reversa simultaneamente
            if (!estadoIniciado) {
                estadoIniciado = true;
                direccionDerecha();
                traccionReversa(P.velocidadParking);
            }
            if (tEnEstado >= (unsigned long)P.tReversaGiroMs) {
                traccionDetener();
                direccionDetener();
                cambiarEstado(TC_FRENO_4);
            }
            break;

        case TC_FRENO_4:
            if (tEnEstado >= 500) {
                cambiarEstado(TC_ENDEREZAR);
            }
            break;

        case TC_ENDEREZAR:
            // Calcular cuanto girar para centrar, no bloqueante
            if (!estadoIniciado) {
                estadoIniciado = true;
                int centro     = P.tiempoDireccionTope / 2;
                int diferencia = centro - posicionDireccion;
                durEnderezarMs = (unsigned long)abs(diferencia);
                if      (diferencia > 0) direccionDerecha();
                else if (diferencia < 0) direccionIzquierda();
                // Si diferencia == 0, durEnderezarMs = 0, sale en seguida
            }
            if (tEnEstado >= durEnderezarMs) {
                direccionDetener();
                posicionDireccion = P.tiempoDireccionTope / 2;
                cambiarEstado(TC_FRENO_5);
            }
            break;

        case TC_FRENO_5:
            if (tEnEstado >= 500) {
                cambiarEstado(TC_REVERSA_RECTA);
            }
            break;

        case TC_REVERSA_RECTA:
            if (!estadoIniciado) {
                estadoIniciado = true;
                traccionReversa(P.velocidadParking);
            }
            if (tEnEstado >= (unsigned long)P.tReversaRectaMs) {
                traccionDetener();
                cambiarEstado(TC_FIN);
            }
            break;

        case TC_FIN:
            modoActual   = ESTACIONADO;
            estadoRutina = NINGUNO;
            modoInter    = INT_AMBAS_FIJAS;
            break;

        default:
            break;
    }
}


// ─── PARSER DE COMANDOS ──────────────────────────────────────────────────────

void procesarComando(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    tUltimoComando = millis();

    // STOP siempre tiene prioridad
    if (cmd == "STOP" || cmd == "ESTOP" || cmd == "stop") {
        pararTodo();
        Serial.println("STOP");
        return;
    }

    // Comandos de prueba de LEDs (modo MANUAL o ESTACIONADO)
    if (cmd.startsWith("LED:")) {
        if      (cmd == "LED:FRENO:1")    digitalWrite(PIN_LED_FRENO, HIGH);
        else if (cmd == "LED:FRENO:0")    { if (pwmTraccionActual > 0) digitalWrite(PIN_LED_FRENO, LOW); }
        else if (cmd == "LED:INTER:IZQ")  modoInter = INT_IZQ;
        else if (cmd == "LED:INTER:DER")  modoInter = INT_DER;
        else if (cmd == "LED:INTER:AMBAS") modoInter = INT_AMBAS;
        else if (cmd == "LED:OFF")        modoInter = INT_OFF;
        return;
    }

    // SET parametro en runtime
    if (cmd.startsWith("SET:")) {
        String par  = cmd.substring(4);
        int    sep  = par.indexOf('=');
        if (sep < 0) return;
        String clave = par.substring(0, sep);
        int    valor = par.substring(sep + 1).toInt();
        if      (clave == "velocidadAvance")     P.velocidadAvance     = valor;
        else if (clave == "velocidadReversa")    P.velocidadReversa    = valor;
        else if (clave == "velocidadParking")    P.velocidadParking    = valor;
        else if (clave == "velocidadDireccion")  P.velocidadDireccion  = valor;
        else if (clave == "tiempoDireccionTope") P.tiempoDireccionTope = valor;
        else if (clave == "kickPwm")             P.kickPwm             = valor;
        else if (clave == "kickMs")              P.kickMs              = valor;
        else if (clave == "tFrenoInicialMs")     P.tFrenoInicialMs     = valor;
        else if (clave == "tAvance1Ms")          P.tAvance1Ms          = valor;
        else if (clave == "tAvance2Ms")          P.tAvance2Ms          = valor;
        else if (clave == "tGiroIzqMs")          P.tGiroIzqMs          = valor;
        else if (clave == "tReversaGiroMs")      P.tReversaGiroMs      = valor;
        else if (clave == "tReversaRectaMs")     P.tReversaRectaMs     = valor;
        else if (clave == "distCarroCm")         P.distCarroCm         = valor;
        else if (clave == "distHuecoCm")         P.distHuecoCm         = valor;
        else if (clave == "distBanquetaCm")      P.distBanquetaCm      = valor;
        else if (clave == "distLateralMinCm")    P.distLateralMinCm    = valor;
        else if (clave == "largoCarroCm")        P.largoCarroCm        = valor;
        else if (clave == "margenHuecoCm")       P.margenHuecoCm       = valor;
        else if (clave == "cmPorSegundo")        P.cmPorSegundo        = valor;
        else if (clave == "muestrasEstables")    P.muestrasEstables    = valor;
        Serial.print("SET "); Serial.print(clave);
        Serial.print("="); Serial.println(valor);
        return;
    }

    // GET: fuerza telemetria inmediata
    if (cmd == "GET") {
        enviarTelemetria();
        return;
    }

    // Iniciar TEST CIEGO (Fase A)
    if (cmd == "TC") {
        pararTodo();
        modoActual    = TEST_CIEGO;
        huecoMedidoCm = 0;
        cambiarEstado(TC_INICIO);
        Serial.println("TEST_CIEGO iniciado");
        return;
    }

    // Iniciar ESTACIONAR DERECHA (Fase B - pendiente)
    if (cmd == "PR") {
        Serial.println("PR recibido - Fase B pendiente de implementar");
        return;
    }

    // Comandos WASD solo en modo MANUAL
    if (modoActual != MANUAL) return;

    if      (cmd == "W" || cmd == "w") traccionAvanzar(P.velocidadAvance);
    else if (cmd == "S" || cmd == "s") traccionReversa(P.velocidadReversa);
    else if (cmd == "A" || cmd == "a") direccionIzquierda();
    else if (cmd == "D" || cmd == "d") direccionDerecha();
    else if (cmd == "X" || cmd == "x") { traccionDetener(); direccionDetener(); }
    else if (cmd == "C" || cmd == "c") direccionDetener();
}


// ─── SETUP ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    // Traccion
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    ledcSetup(CH_TRACCION, PWM_FRECUENCIA, PWM_BITS);
    ledcAttachPin(PIN_ENB, CH_TRACCION);

    // Direccion
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    ledcSetup(CH_DIRECCION, PWM_FRECUENCIA, PWM_BITS);
    ledcAttachPin(PIN_ENA, CH_DIRECCION);

    // Ultrasonicos
    pinMode(PIN_TRIG_DER, OUTPUT); pinMode(PIN_ECHO_DER, INPUT);
    pinMode(PIN_TRIG_IZQ, OUTPUT); pinMode(PIN_ECHO_IZQ, INPUT);
    pinMode(PIN_TRIG_TRA, OUTPUT); pinMode(PIN_ECHO_TRA, INPUT);
    pinMode(PIN_TRIG_FRE, OUTPUT); pinMode(PIN_ECHO_FRE, INPUT);

    // LEDs
    pinMode(PIN_LED_IZQ,   OUTPUT);
    pinMode(PIN_LED_DER,   OUTPUT);
    pinMode(PIN_LED_FRENO, OUTPUT);

    // UART2 a Raspberry Pi
    Serial2.begin(115200, SERIAL_8N1, 16, 17);

    // Calibrar direccion al tope izquierdo para tener origen conocido
    Serial.println("Calibrando direccion...");
    calibrarDireccion();
    Serial.println("Direccion calibrada");

    // WiFi AP
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    servidor.begin();
    Serial.print("AP activo: ");
    Serial.print(WIFI_SSID);
    Serial.print("  IP: ");
    Serial.println(WiFi.softAPIP());
}


// ─── LOOP ────────────────────────────────────────────────────────────────────

void loop() {
    unsigned long ahora = millis();

    // Aceptar cliente TCP nuevo si no hay uno conectado
    if (!cliente || !cliente.connected()) {
        cliente = servidor.available();
    }

    // Leer bytes del cliente TCP
    if (cliente && cliente.connected()) {
        while (cliente.available()) {
            char c = cliente.read();
            if (c == '\n') {
                if (bufferTCP.length() > 0) procesarComando(bufferTCP);
                bufferTCP = "";
            } else if (c != '\r') {
                if (bufferTCP.length() < BUFFER_MAX) bufferTCP += c;
            }
        }
    }

    // Leer bytes de UART2 (Raspberry Pi)
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') {
            if (bufferUART.length() > 0) procesarComando(bufferUART);
            bufferUART = "";
        } else if (c != '\r') {
            if (bufferUART.length() < BUFFER_MAX) bufferUART += c;
        }
    }

    // Timeout modo MANUAL: detener traccion si no llegan comandos
    if (modoActual == MANUAL && pwmTraccionActual > 0) {
        if (ahora - tUltimoComando > TIMEOUT_MANUAL_MS) {
            traccionDetener();
        }
    }

    // Avanzar maquina de estados activa
    if (modoActual == TEST_CIEGO) {
        loopTestCiego();
    }

    // Leer un sensor por tick (rotacion)
    if (ahora - tUltimoSensor >= INTERVALO_SENSOR_MS) {
        tUltimoSensor = ahora;
        leerSensorTurno();
    }

    // Actualizar LEDs (no bloqueante)
    actualizarLEDs();

    // Telemetria cada 200 ms
    if (ahora - tUltimaTelemetria >= INTERVALO_TELEMETRIA_MS) {
        tUltimaTelemetria = ahora;
        enviarTelemetria();
    }
}
