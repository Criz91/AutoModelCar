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
    // Velocidades (0-255) — minimo funcional en piso: ~190
    int velocidadAvance    = 220;
    int velocidadReversa   = 230; // La reversa tiende a ser mas lenta, compensar
    int velocidadParking   = 220; // Para Fase B
    int velocidadDireccion = 208;

    // Direccion
    int tiempoDireccionTope = 355; // ms de un tope al otro (calibrado)

    // Tiempos Fase A - test ciego (ms)
    // El carro mide 31 cm largo x 20 cm ancho
    int tFrenoInicialMs  = 5000;  // Pausa inicial de arranque
    int tAvance1Ms       = 1000;  // Avance 1: simula encontrar el primer carro
    int tAvance2Ms       = 1000;  // Avance 2: sensor queda en mitad del hueco
    int tGiroIzqMs       = 500;   // Abre a la izq + avanza (posiciona para reversa)
    int tReversaGiroMs   = 1000;  // Reversa girando der (mete la cola)
    int tEnderezarMs     = 500;   // Reversa con leve giro der (alinea sin centro real)
    int tReversaRectaMs  = 1000;  // Reversa recta (entra al cajon)
    int tPausaMs         = 10000; // Pausas de observacion (debug; reducir en competencia)

    // Fase B - sensores (cm)
    int distCarroCm      = 15;
    int distHuecoCm      = 25;
    int distBanquetaCm   = 5;
    int distLateralMinCm = 3;
    int largoCarroCm     = 31;  // largo real del vehiculo
    int margenHuecoCm    = 5;
    int cmPorSegundo     = 30;  // calibrar en Fase A
    int muestrasEstables = 3;
};

enum Modo {
    MANUAL,
    TEST_CIEGO,
    ESTACIONAR_DER,
    ESTACIONADO,
    ABORTADO,
    PARADA_SENIAL  // Señal de STOP: 6s parado con intermitentes, luego vuelve a MANUAL
};

enum EstadoRutina {
    NINGUNO,

    // Fase A - Test ciego (secuencia con pausas de observacion)
    TC_INICIO,
    TC_AVANCE_1,
    TC_PAUSA_1,
    TC_AVANCE_2,
    TC_PAUSA_2,
    TC_GIRO_IZQ,
    TC_PAUSA_3,
    TC_REVERSA_GIRO,
    TC_PAUSA_4,
    TC_ENDEREZAR,
    TC_PAUSA_5,
    TC_REVERSA_RECTA,
    TC_FIN,

    // Fase B - Estacionamiento con sensores (ESTACIONAR_DER)
    PB_INICIO,           // Pausa inicial 5s
    PB_BUSCA_CARRO,      // Avanza buscando un carro a la derecha
    PB_DETECTADO_CARRO,  // Carro detectado, sigue avanzando buscando el hueco
    PB_MIDE_HUECO,       // Midiendo largo del hueco (cronometrando)
    PB_VERIFICA_CABE,    // Verifica si el hueco es suficiente (largoCarroCm + margenHuecoCm)
    PB_AVANCE_EXTRA,     // Avanza medio carro para alinear eje trasero con inicio del hueco
    PB_GIRO_IZQ,         // Abre izq + avanza (mismo que TC_GIRO_IZQ)
    PB_REVERSA_GIRO,     // Gira der + reversa (mismo que TC_REVERSA_GIRO)
    PB_ENDEREZAR,        // Leve der + reversa (mismo que TC_ENDEREZAR)
    PB_REVERSA_BANQUETA, // Reversa recta hasta distTra <= distBanquetaCm
    PB_CHEQUEO_LATERAL,  // Verifica si un lateral esta muy cerca de un carro vecino
    PB_MANIOBRA_CORR,    // Saca un poco y ajusta si quedo muy pegado
    PB_ESTACIONADO,      // Terminado correctamente
    PB_ABORTADO          // Hueco insuficiente o error
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

// Fase B - helpers de debounce y maniobra
unsigned long tCondicionInicio = 0;   // Cuando se detecto la condicion por primera vez
bool          condicionActiva  = false;
unsigned long tInicioHueco     = 0;   // Marca de tiempo inicio del hueco (PB_MIDE_HUECO)
unsigned long durAvanceExtraMs = 0;   // Duracion calculada para PB_AVANCE_EXTRA
int           intentosCorr     = 0;   // Intentos de maniobra correctiva (max 2)
bool          corrIzqLado      = false; // true=muy pegado izq, false=muy pegado der

// PARADA_SENIAL
unsigned long tInicioParada = 0;

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
    // Pines invertidos para corregir el giro físico
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    ledcWrite(CH_DIRECCION, P.velocidadDireccion);
}

void direccionDerecha() {
    actualizarPosicion();
    ladoDireccion    = DIR_DER;
    tInicioDireccion = millis();
    // Pines invertidos para corregir el giro físico
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(CH_DIRECCION, P.velocidadDireccion);
}

void direccionDetener() {
    actualizarPosicion();
    
    // Micro-kick de rescate para desatascar los engranes
    if (ladoDireccion == DIR_IZQ) {
        digitalWrite(PIN_IN1, HIGH); // Pulso contrario (Derecha)
        digitalWrite(PIN_IN2, LOW);
        ledcWrite(CH_DIRECCION, 255); // Fuerza maxima
        delay(40); // Tiempo infimo, solo para soltar el engrane
    } else if (ladoDireccion == DIR_DER) {
        digitalWrite(PIN_IN1, LOW);  // Pulso contrario (Izquierda)
        digitalWrite(PIN_IN2, HIGH);
        ledcWrite(CH_DIRECCION, 255);
        delay(40);
    }

    ladoDireccion = DIR_STOP;
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(CH_DIRECCION, 0);
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
            // Pines fisicamente invertidos respecto a su nombre: PIN_LED_IZQ=10 es el fisico DER
            digitalWrite(PIN_LED_IZQ, (modoInter == INT_DER || modoInter == INT_AMBAS) ? (b ? HIGH : LOW) : LOW);
            digitalWrite(PIN_LED_DER, (modoInter == INT_IZQ || modoInter == INT_AMBAS) ? (b ? HIGH : LOW) : LOW);
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
        case PARADA_SENIAL:  return "PARADA_SENIAL";
    }
    return "?";
}

const char* nombreEstado() {
    switch (estadoRutina) {
        case NINGUNO:          return "NINGUNO";
        case TC_INICIO:        return "TC_INICIO";
        case TC_AVANCE_1:      return "TC_AVANCE_1";
        case TC_PAUSA_1:       return "TC_PAUSA_1";
        case TC_AVANCE_2:      return "TC_AVANCE_2";
        case TC_PAUSA_2:       return "TC_PAUSA_2";
        case TC_GIRO_IZQ:      return "TC_GIRO_IZQ";
        case TC_PAUSA_3:       return "TC_PAUSA_3";
        case TC_REVERSA_GIRO:  return "TC_REVERSA_GIRO";
        case TC_PAUSA_4:       return "TC_PAUSA_4";
        case TC_ENDEREZAR:     return "TC_ENDEREZAR";
        case TC_PAUSA_5:       return "TC_PAUSA_5";
        case TC_REVERSA_RECTA: return "TC_REVERSA_RECTA";
        case TC_FIN:           return "TC_FIN";
        case PB_INICIO:           return "PB_INICIO";
        case PB_BUSCA_CARRO:      return "PB_BUSCA_CARRO";
        case PB_DETECTADO_CARRO:  return "PB_DETECTADO_CARRO";
        case PB_MIDE_HUECO:       return "PB_MIDE_HUECO";
        case PB_VERIFICA_CABE:    return "PB_VERIFICA_CABE";
        case PB_AVANCE_EXTRA:     return "PB_AVANCE_EXTRA";
        case PB_GIRO_IZQ:         return "PB_GIRO_IZQ";
        case PB_REVERSA_GIRO:     return "PB_REVERSA_GIRO";
        case PB_ENDEREZAR:        return "PB_ENDEREZAR";
        case PB_REVERSA_BANQUETA: return "PB_REVERSA_BANQUETA";
        case PB_CHEQUEO_LATERAL:  return "PB_CHEQUEO_LATERAL";
        case PB_MANIOBRA_CORR:    return "PB_MANIOBRA_CORR";
        case PB_ESTACIONADO:      return "PB_ESTACIONADO";
        case PB_ABORTADO:         return "PB_ABORTADO";
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

        // --- Pausa inicial 5 s (freno on por regla general) ---
        case TC_INICIO:
            if (tEnEstado >= (unsigned long)P.tFrenoInicialMs)
                cambiarEstado(TC_AVANCE_1);
            break;

        // --- Avance 1: simula encontrar el primer carro ---
        case TC_AVANCE_1:
            if (!estadoIniciado) { estadoIniciado = true; traccionAvanzar(P.velocidadAvance); }
            if (tEnEstado >= (unsigned long)P.tAvance1Ms) { traccionDetener(); cambiarEstado(TC_PAUSA_1); }
            break;

        case TC_PAUSA_1:
            if (tEnEstado >= (unsigned long)P.tPausaMs) cambiarEstado(TC_AVANCE_2);
            break;

        // --- Avance 2: sensor queda aprox en mitad del hueco, enciende intermitentes ---
        case TC_AVANCE_2:
            if (!estadoIniciado) {
                estadoIniciado = true;
                modoInter = INT_AMBAS;
                traccionAvanzar(P.velocidadAvance);
            }
            if (tEnEstado >= (unsigned long)P.tAvance2Ms) { traccionDetener(); cambiarEstado(TC_PAUSA_2); }
            break;

        case TC_PAUSA_2:
            if (tEnEstado >= (unsigned long)P.tPausaMs) cambiarEstado(TC_GIRO_IZQ);
            break;

        // --- Giro izquierda + avance: abre el carro para quedar en bateria ---
        case TC_GIRO_IZQ:
            if (!estadoIniciado) {
                estadoIniciado = true;
                direccionIzquierda();
                traccionAvanzar(P.velocidadAvance);
            }
            if (tEnEstado >= (unsigned long)P.tGiroIzqMs) {
                traccionDetener();
                direccionDetener();
                cambiarEstado(TC_PAUSA_3);
            }
            break;

        case TC_PAUSA_3:
            if (tEnEstado >= (unsigned long)P.tPausaMs) cambiarEstado(TC_REVERSA_GIRO);
            break;

        // --- Giro derecha + reversa: mete la cola al cajon ---
        case TC_REVERSA_GIRO:
            if (!estadoIniciado) {
                estadoIniciado = true;
                direccionDerecha();
                traccionReversa(P.velocidadReversa);
            }
            if (tEnEstado >= (unsigned long)P.tReversaGiroMs) {
                traccionDetener();
                direccionDetener();
                cambiarEstado(TC_PAUSA_4);
            }
            break;

        case TC_PAUSA_4:
            if (tEnEstado >= (unsigned long)P.tPausaMs) cambiarEstado(TC_ENDEREZAR);
            break;

        // --- Leve giro der + reversa: alinea (el resorte ayuda a centrar) ---
        case TC_ENDEREZAR:
            if (!estadoIniciado) {
                estadoIniciado = true;
                direccionDerecha();                      // Giro leve a la derecha
                traccionReversa(P.velocidadReversa);
            }
            // Suelta la direccion a los 150 ms; el resorte centra el resto
            if (tEnEstado >= 150 && ladoDireccion != DIR_STOP) {
                direccionDetener();
            }
            if (tEnEstado >= (unsigned long)P.tEnderezarMs) {
                traccionDetener();
                cambiarEstado(TC_PAUSA_5);
            }
            break;

        case TC_PAUSA_5:
            if (tEnEstado >= (unsigned long)P.tPausaMs) cambiarEstado(TC_REVERSA_RECTA);
            break;

        // --- Reversa recta: entra al cajon hasta la banqueta ---
        case TC_REVERSA_RECTA:
            if (!estadoIniciado) { estadoIniciado = true; traccionReversa(P.velocidadReversa); }
            if (tEnEstado >= (unsigned long)P.tReversaRectaMs) { traccionDetener(); cambiarEstado(TC_FIN); }
            break;

        // --- Fin: estacionado ---
        case TC_FIN:
            modoActual   = ESTACIONADO;
            estadoRutina = NINGUNO;
            modoInter    = INT_AMBAS_FIJAS;
            break;

        default: break;
    }
}


// ─── PARADA SEÑAL DE STOP ────────────────────────────────────────────────────

void loopParadaSenial() {
    if (millis() - tInicioParada >= 6000UL) {
        modoInter  = INT_OFF;
        modoActual = MANUAL;
        Serial.println("PARADA_SENIAL: fin, volviendo a MANUAL");
    }
}


// ─── FASE B: ESTACIONAMIENTO CON SENSORES ────────────────────────────────────

void loopEstacionarDer() {
    unsigned long tEnEstado = millis() - tInicioEstado;

    switch (estadoRutina) {

        // --- Pausa inicial 5s (freno on por regla general) ---
        case PB_INICIO:
            if (tEnEstado >= 5000UL) {
                condicionActiva = false;
                cambiarEstado(PB_BUSCA_CARRO);
            }
            break;

        // --- Avanza buscando un carro a la derecha ---
        // Necesita muestrasEstables * 250ms de lectura estable para confirmar
        case PB_BUSCA_CARRO:
            if (!estadoIniciado) { estadoIniciado = true; traccionAvanzar(P.velocidadParking); }
            if (distDer <= (float)P.distCarroCm) {
                if (!condicionActiva) { condicionActiva = true; tCondicionInicio = millis(); }
                else if (millis() - tCondicionInicio >= (unsigned long)(P.muestrasEstables * 250)) {
                    condicionActiva = false;
                    cambiarEstado(PB_DETECTADO_CARRO);
                }
            } else {
                condicionActiva = false;
            }
            if (tEnEstado >= 30000UL) { pararTodo(); modoActual = ABORTADO; }
            break;

        // --- Carro detectado; enciende intermitentes y sigue buscando el hueco ---
        case PB_DETECTADO_CARRO:
            if (!estadoIniciado) { estadoIniciado = true; modoInter = INT_AMBAS; }
            if (distDer > (float)P.distHuecoCm) {
                if (!condicionActiva) { condicionActiva = true; tCondicionInicio = millis(); }
                else if (millis() - tCondicionInicio >= (unsigned long)(P.muestrasEstables * 250)) {
                    condicionActiva = false;
                    tInicioHueco = millis();
                    cambiarEstado(PB_MIDE_HUECO);
                }
            } else {
                condicionActiva = false;
            }
            if (tEnEstado >= 15000UL) { pararTodo(); modoActual = ABORTADO; }
            break;

        // --- Mide el largo del hueco mientras avanza ---
        // La traccion sigue activa desde PB_DETECTADO_CARRO, no es necesario reactivarla
        case PB_MIDE_HUECO: {
            // Timeout dinamico: un poco mas que el hueco minimo necesario
            unsigned long timeoutHueco = (unsigned long)((P.largoCarroCm + P.margenHuecoCm + 20) * 1000.0f / P.cmPorSegundo);
            if (distDer <= (float)P.distCarroCm) {
                if (!condicionActiva) { condicionActiva = true; tCondicionInicio = millis(); }
                else if (millis() - tCondicionInicio >= (unsigned long)(P.muestrasEstables * 250)) {
                    huecoMedidoCm = (float)(millis() - tInicioHueco) * P.cmPorSegundo / 1000.0f;
                    traccionDetener();
                    condicionActiva = false;
                    cambiarEstado(PB_VERIFICA_CABE);
                }
            } else {
                condicionActiva = false;
            }
            // Si el hueco es mas largo que el maximo esperado, asumir que cabe
            if (tEnEstado >= timeoutHueco) {
                huecoMedidoCm = (float)tEnEstado * P.cmPorSegundo / 1000.0f;
                traccionDetener();
                cambiarEstado(PB_VERIFICA_CABE);
            }
            break;
        }

        // --- Verifica si el hueco es suficiente para el vehiculo ---
        case PB_VERIFICA_CABE:
            if (huecoMedidoCm >= (float)(P.largoCarroCm + P.margenHuecoCm)) {
                // Calcula cuanto avanzar para que el eje trasero quede al inicio del hueco
                durAvanceExtraMs = (unsigned long)((P.largoCarroCm / 2.0f) / P.cmPorSegundo * 1000.0f);
                Serial.print("Hueco OK: "); Serial.print(huecoMedidoCm); Serial.println(" cm");
                cambiarEstado(PB_AVANCE_EXTRA);
            } else {
                modoActual     = ABORTADO;
                estadoRutina   = PB_ABORTADO;
                tInicioEstado  = millis();
                estadoIniciado = false;
                modoInter      = INT_AMBAS;
                Serial.print("Hueco insuficiente: "); Serial.print(huecoMedidoCm); Serial.println(" cm");
            }
            break;

        // --- Avanza medio carro para alinear el eje trasero con el inicio del hueco ---
        case PB_AVANCE_EXTRA:
            if (!estadoIniciado) { estadoIniciado = true; traccionAvanzar(P.velocidadParking); }
            if (tEnEstado >= durAvanceExtraMs) {
                traccionDetener();
                cambiarEstado(PB_GIRO_IZQ);
            }
            break;

        // --- Giro izquierda + avance: abre el carro para entrar en bateria ---
        case PB_GIRO_IZQ:
            if (!estadoIniciado) {
                estadoIniciado = true;
                direccionIzquierda();
                traccionAvanzar(P.velocidadParking);
            }
            if (tEnEstado >= (unsigned long)P.tGiroIzqMs) {
                traccionDetener();
                direccionDetener();
                cambiarEstado(PB_REVERSA_GIRO);
            }
            break;

        // --- Giro derecha + reversa: mete la cola al cajon ---
        case PB_REVERSA_GIRO:
            if (!estadoIniciado) {
                estadoIniciado = true;
                direccionDerecha();
                traccionReversa(P.velocidadParking);
            }
            if (tEnEstado >= (unsigned long)P.tReversaGiroMs) {
                traccionDetener();
                direccionDetener();
                cambiarEstado(PB_ENDEREZAR);
            }
            break;

        // --- Leve giro der 150ms + reversa; el resorte centra el resto ---
        case PB_ENDEREZAR:
            if (!estadoIniciado) {
                estadoIniciado = true;
                direccionDerecha();
                traccionReversa(P.velocidadParking);
            }
            if (tEnEstado >= 150 && ladoDireccion != DIR_STOP) {
                direccionDetener();
            }
            if (tEnEstado >= (unsigned long)P.tEnderezarMs) {
                traccionDetener();
                cambiarEstado(PB_REVERSA_BANQUETA);
            }
            break;

        // --- Reversa recta hasta llegar a la banqueta (sensor trasero) ---
        case PB_REVERSA_BANQUETA:
            if (!estadoIniciado) { estadoIniciado = true; traccionReversa(P.velocidadParking); }
            if (distTra <= (float)P.distBanquetaCm) {
                traccionDetener();
                cambiarEstado(PB_CHEQUEO_LATERAL);
            }
            if (tEnEstado >= 8000UL) {  // Red de seguridad si el sensor no responde
                traccionDetener();
                cambiarEstado(PB_CHEQUEO_LATERAL);
            }
            break;

        // --- Verifica que no quedamos muy pegados a un carro vecino ---
        case PB_CHEQUEO_LATERAL:
            if (distIzq < (float)P.distLateralMinCm && intentosCorr < 2) {
                corrIzqLado = true;
                intentosCorr++;
                cambiarEstado(PB_MANIOBRA_CORR);
            } else if (distDer < (float)P.distLateralMinCm && intentosCorr < 2) {
                corrIzqLado = false;
                intentosCorr++;
                cambiarEstado(PB_MANIOBRA_CORR);
            } else {
                modoActual     = ESTACIONADO;
                estadoRutina   = PB_ESTACIONADO;
                tInicioEstado  = millis();
                estadoIniciado = false;
                modoInter      = INT_AMBAS_FIJAS;
                Serial.println("ESTACIONADO correctamente");
            }
            break;

        // --- Sale un poco y corrige el angulo segun el lado mas cercano ---
        case PB_MANIOBRA_CORR:
            if (!estadoIniciado) { estadoIniciado = true; traccionAvanzar(P.velocidadParking); }
            if (tEnEstado >= 500UL) {
                traccionDetener();
                if (corrIzqLado) direccionDerecha();   // Muy pegado izq → girar der
                else             direccionIzquierda(); // Muy pegado der → girar izq
                delay(150);
                direccionDetener();
                cambiarEstado(PB_REVERSA_BANQUETA);
            }
            break;

        case PB_ESTACIONADO:
        case PB_ABORTADO:
            break;

        default: break;
    }
}


// ─── PARSER DE COMANDOS ──────────────────────────────────────────────────────

void procesarComando(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    tUltimoComando = millis();

    // ESTOP: parada de emergencia permanente (GUI, operador)
    if (cmd == "ESTOP" || cmd == "estop") {
        pararTodo();
        Serial.println("ESTOP");
        return;
    }

    // STOP: señal de transito — para 6s con intermitentes, luego vuelve a MANUAL
    if (cmd == "STOP" || cmd == "stop") {
        traccionDetener();
        direccionDetener();
        estadoRutina   = NINGUNO;
        modoActual     = PARADA_SENIAL;
        modoInter      = INT_AMBAS;
        tInicioParada  = millis();
        estadoIniciado = false;
        Serial.println("PARADA_SENIAL: 6s intermitentes");
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
        else if (clave == "tFrenoInicialMs")     P.tFrenoInicialMs     = valor;
        else if (clave == "tAvance1Ms")          P.tAvance1Ms          = valor;
        else if (clave == "tAvance2Ms")          P.tAvance2Ms          = valor;
        else if (clave == "tGiroIzqMs")          P.tGiroIzqMs          = valor;
        else if (clave == "tReversaGiroMs")      P.tReversaGiroMs      = valor;
        else if (clave == "tEnderezarMs")        P.tEnderezarMs        = valor;
        else if (clave == "tReversaRectaMs")     P.tReversaRectaMs     = valor;
        else if (clave == "tPausaMs")            P.tPausaMs            = valor;
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

    // Iniciar ESTACIONAR DERECHA (Fase B)
    if (cmd == "PR") {
        pararTodo();
        modoActual      = ESTACIONAR_DER;
        huecoMedidoCm   = 0;
        intentosCorr    = 0;
        condicionActiva = false;
        cambiarEstado(PB_INICIO);
        Serial.println("ESTACIONAR_DER iniciado");
        return;
    }

    //Comandos para el Test de Topes Seguro 
    if (cmd == "TEST_DIR:IZQ") {
        Serial.println("Test Tope Izquierdo...");
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        ledcWrite(CH_DIRECCION, P.velocidadDireccion);
        delay(P.tiempoDireccionTope); // Mueve exacto el tiempo del slider
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        ledcWrite(CH_DIRECCION, 0);
        return;
    }
    
    if (cmd == "TEST_DIR:DER") {
        Serial.println("Test Tope Derecho...");
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        ledcWrite(CH_DIRECCION, P.velocidadDireccion);
        delay(P.tiempoDireccionTope); // Mueve exacto el tiempo del slider
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        ledcWrite(CH_DIRECCION, 0);
        return;
    }
    

    // Comandos WASD (+ TX) solo en modo MANUAL o fuerzan retorno a MANUAL
    if (cmd == "W" || cmd == "w" || cmd == "S" || cmd == "s" ||
        cmd == "A" || cmd == "a" || cmd == "D" || cmd == "d" ||
        cmd == "X" || cmd == "x" || cmd == "TX"|| cmd == "tx" ||
        cmd == "C" || cmd == "c") {

        // Durante una parada de señal no interrumpir con movimiento
        if (modoActual == PARADA_SENIAL) return;

        if (modoActual != MANUAL) {
            Serial.println("Intervencion humana: Control manual retomado");
            pararTodo(); // Reinicia el estado y modo a MANUAL
        }
        
        if      (cmd == "W" || cmd == "w")  traccionAvanzar(P.velocidadAvance);
        else if (cmd == "S" || cmd == "s")  traccionReversa(P.velocidadReversa);
        else if (cmd == "A" || cmd == "a")  direccionIzquierda();
        else if (cmd == "D" || cmd == "d")  direccionDerecha();
        else if (cmd == "X" || cmd == "x")  { traccionDetener(); direccionDetener(); }
        else if (cmd == "TX" || cmd == "tx") traccionDetener();
        else if (cmd == "C" || cmd == "c")  direccionDetener();
    }
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

    // --- NUEVO: Arranque pasivo (Sin golpes) ---
    // Asumimos que tú pusiste las llantas derechas con la mano antes de encender el carro.
    posicionDireccion = P.tiempoDireccionTope / 2; // Actualizar tracker al centro virtual
    Serial.println("Arranque: Direccion asumida al centro, motores apagados.");
    // -------------------------------------------

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
    if (modoActual == TEST_CIEGO)     loopTestCiego();
    if (modoActual == ESTACIONAR_DER) loopEstacionarDer();
    if (modoActual == PARADA_SENIAL)  loopParadaSenial();

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
