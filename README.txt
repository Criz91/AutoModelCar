#PREVIO
Se necesita crear un entorno virtual para instalar las dependencias
y que no se instalen directamente en tu computador.
Ejecuta:
    python -m venv venv
    venv\Scripts\activate
    pip install -r requirements.txt

Y para ejecutar la interfaz de control:
    python control_gui.py

Antes de abrir el GUI hay que conectarse a la red WiFi que crea el
ESP32 (SSID "ESP32_PRUEBA", contrasena "123456785"). El GUI abre el
WebSocket en ws://192.168.4.1/ws.


#Hardware del carro
Chasis con puente H, motor de traccion atras, motor de direccion
adelante, ESP32-S3 N16R8 con WiFi en modo Access Point, tres
ultrasonicos HC-SR04 (derecho, izquierdo, trasero) montados a media
altura del costado, dos LEDs como intermitentes (hazards).
Medidas reales del carro: 29 cm de largo x 14 cm de ancho x 25 cm de
alto. Esto importa porque la rutina de estacionamiento usa estas
dimensiones para calcular si el carro cabe en un hueco y para saber
cuanto tiene que avanzar antes de empezar a meterse de reversa.


#MIKE/ESP_MOVIMIENTO/ESP_MOVIMIENTO.ino
Es el firmware del ESP32-S3 escrito en C++ con el framework de Arduino.
Hace tres cosas principales:

  1. Levanta una red WiFi en modo Access Point y abre un servidor
     WebSocket en /ws. Por ese WebSocket recibe comandos del GUI
     (W, S, A, D, X, C, PR, PL, T, H, CAL, ESTOP) y manda telemetria
     200 ms (modo, sub-estado, lecturas R/L/B, posicion estimada de
     la direccion).

  2. Controla los dos motores con el puente H. Cada motor recibe
     una senal PWM por LEDC para la velocidad y dos pines digitales
     para el sentido de giro. El de traccion va atras (avanza /
     retrocede), el de direccion va adelante (gira las llantas).

  3. Implementa la maniobra autonoma de estacionamiento en bateria
     mediante una maquina de estados:
         AVANCE_INICIAL    -> recorre 1 m recto desde la salida
         MEDIR_HUECO       -> con el sensor lateral, integra la
                              longitud del hueco hasta que sea
                              mayor que ancho_carro + margen
         AVANZAR_PASA_HUECO-> avanza medio largo de carro mas para
                              que el eje trasero quede en el borde
         STOP_Y_GIRAR      -> gira la direccion al tope hacia el
                              lado del estacionamiento
         REVERSA_GIRO      -> entra de reversa en diagonal
         STOP_Y_ENDEREZAR  -> centra la direccion
         REVERSA_RECTA     -> reversa lenta hasta antes de pegar
                              con la banqueta
         ESTACIONADO       -> hazards fijas, fin
     En cualquier paso, si un sensor lateral detecta a un vecino
     muy cerca, o si el watchdog de 60 s se cumple, se va a
     ABORTADO con doble flash.


#control_gui.py
GUI en Python (Tkinter) que se conecta al WebSocket del ESP32 y
permite tres cosas:

  - Control manual del carro con teclas WASD o con botones en pantalla.
    W = adelante, S = reversa, A = direccion izquierda, D = direccion
    derecha, X = parar traccion, C = centrar direccion, Esc = paro de
    emergencia. Como el motor de direccion no es servo sino DC con
    puente H, A y D solo giran las llantas: para avanzar girando hay
    que tener W y A presionadas a la vez (igual que en un coche real).

  - Arrancar la maniobra automatica con los botones "Estacionar
    DERECHA" (PR) o "Estacionar IZQUIERDA" (PL). Cualquier tecla
    durante la maniobra cancela y manda al carro a ABORT.

  - Ajustar todos los parametros de calibracion en vivo con sliders
    (velocidad PWM, dimensiones del carro, distancias de deteccion,
    margen de banqueta, tiempo de reversa con giro, kick-start, etc).
    Cada slider manda un comando "SET:clave=valor" al firmware, asi
    que se puede afinar el comportamiento sin reflashear el ESP32.

El GUI tambien muestra en vivo las lecturas de los tres ultrasonicos
en cm, la posicion estimada de la direccion y el estado actual del
modo automatico.


#Que es el PWM y como se usa aqui
PWM (Pulse Width Modulation) es una tecnica para simular un voltaje
intermedio prendiendo y apagando un pin digital muy rapido. El motor
ve un voltaje promedio igual a Vbateria * (duty / 255), donde el duty
es el porcentaje del tiempo que el pin esta en HIGH dentro de cada
ciclo. En este firmware:

  - Frecuencia: 20 kHz. Esta sobre el rango audible asi que no se oye
    el zumbido del motor.
  - Resolucion: 8 bits. Duty de 0 a 255 (0 = parado, 255 = 100%).
  - Dos canales LEDC independientes: uno para el motor de traccion
    (CH_DRIVE en el pin ENB) y otro para el motor de direccion
    (CH_STEER en el pin ENA). El sentido de giro lo dan los pines
    IN1/IN2 (direccion) e IN3/IN4 (traccion) del puente H.

Importante: a duty bajo (por ejemplo 130/255 = 51%) el motor recibe
poco torque y a veces no logra vencer la friccion estatica del tren
motriz, asi que zumba pero las llantas no giran. Por eso el firmware
hace "kick-start": al entrar a cada estado de movimiento aplica un
PWM alto (240/255) durante 150 ms para arrancar y luego baja al
PWM de crucero. Tanto el PWM de crucero como el del kick-start son
ajustables desde el GUI.


#Como funciona el HC-SR04
El HC-SR04 es un sensor ultrasonico. Manda un pulso de sonido de
40 kHz, espera el eco y mide cuanto tarda en regresar. La distancia
en cm es: tiempo_us * 0.0343 / 2 (la velocidad del sonido es
~343 m/s, dividido entre 2 porque el sonido va y vuelve).

Caracteristicas relevantes para esta aplicacion:

  - Rango efectivo: aproximadamente 2 cm a 400 cm. Por debajo de
    2 cm el eco vuelve antes de que termine el pulso de trigger y
    se pierde.
  - Precision: alrededor de +/- 1 cm en condiciones buenas, hasta
    +/- 3 cm con superficies anguladas o muy lisas.
  - Cono de deteccion: aproximadamente 15 grados de angulo total.
    El sensor NO mide en linea recta, mide dentro de un cono. A
    10 cm de distancia el haz cubre un circulo de ~2.6 cm; a 20 cm,
    ~5 cm; a 50 cm, ~13 cm; a 1 m, ~26 cm. Esto significa que
    cualquier obstaculo dentro de ese cono devuelve eco, no solo lo
    que esta directamente al frente.
  - Frecuencia de muestreo: el firmware lee los tres sensores en
    secuencia con un timeout de 25 ms cada uno, asi que cada sensor
    se actualiza ~13 veces por segundo.
  - Lecturas invalidas: si no hay eco antes del timeout, el firmware
    devuelve 999 cm. Estas lecturas (y las menores a 5 cm, que casi
    siempre son ruido) se descartan antes de meterse al filtro de
    mediana, asi no contaminan el promedio.
  - Filtro: cada sensor pasa por una mediana de 5 muestras para
    rechazar outliers de un solo frame.

Ojo con la altura: como los HC-SR04 estan a media altura del costado
del carro, si los carros vecinos son muy bajos (mas bajos que el
sensor) el cono podria pasar por encima y no haber eco. Hay que
verificar fisicamente la altura del sensor antes de cada prueba.


#Carpeta sketch_jan26a
Carpeta vieja con codigo de Arduino que recibia ordenes por puerto
serial. No se usa en la version actual, queda solo como referencia.


#contro_car.py
Version inicial del control en Python, no se usa.
