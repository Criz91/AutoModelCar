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
     cada 200 ms (modo, sub-estado, lecturas R/L/B, posicion estimada
     de la direccion).

  2. Controla los dos motores con el puente H. Cada motor recibe una
     senal PWM por LEDC para la velocidad y dos pines digitales para
     el sentido de giro. El de traccion va atras (avanza/retrocede),
     el de direccion va adelante (gira las llantas).

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

Comportamiento de los LEDs durante la maniobra:
  - AVANCE_INICIAL: intermitente rapido (modo 2). Es el aviso visual
    de que el carro acaba de arrancar el modo automatico.
  - MEDIR_HUECO: apagados. El carro esta buscando, sin senal aun.
  - Desde que confirma el hueco hasta REVERSA_RECTA: intermitente
    lento (modo 1), igual que un coche real cuando se va a estacionar.
  - ESTACIONADO: ambos LEDs fijos prendidos (modo 3). Asi se queda
    hasta que se mande otro comando o se reinicie el carro.
  - ABORTADO: doble flash rapido (modo 4) como senal de error.


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

  - Ajustar todos los parametros de calibracion en vivo con sliders.
    Cada slider manda un comando "SET:clave=valor" al firmware, asi
    que se puede afinar el comportamiento sin reflashear el ESP32.

El GUI tambien muestra en vivo las lecturas de los tres ultrasonicos
en cm, la posicion estimada de la direccion y el estado actual del
modo automatico.


#Que es el PWM y por que lo usamos
PWM significa Pulse Width Modulation, en espanol "modulacion por
ancho de pulso". Es una tecnica para controlar cuanta energia recibe
un motor sin necesidad de variar el voltaje real de la bateria.

La idea fisica: en lugar de mandarle al motor un voltaje constante
(por ejemplo 5 V todo el tiempo), le mandamos pulsos cuadrados muy
rapidos que se prenden y se apagan. Si la senal esta prendida la
mitad del tiempo y apagada la otra mitad, el motor se comporta como
si recibiera la mitad del voltaje. Si esta prendida un 80% del tiempo,
recibe el equivalente al 80%. El motor "no recibe todo constante, va por pulsos": el
voltaje es siempre todo o nada, lo que cambia es cuanto tiempo dura
prendido en cada ciclo.

Dos cosas definen una senal PWM:

  - Frecuencia: cuantos ciclos por segundo. En este firmware usamos
    20 kHz (20 000 ciclos por segundo, cada ciclo dura 50 microsegundos).
    Esa frecuencia esta arriba del rango que oye una persona, por eso
    el motor no chilla.

  - Duty cycle: el porcentaje del ciclo en el que la senal esta
    prendida. Aqui usamos resolucion de 8 bits, asi que el duty va
    de 0 a 255: 0 = siempre apagado (motor parado), 255 = siempre
    prendido (motor a tope), 128 = mitad del tiempo prendido (mitad
    de potencia).

Como lo usamos en este carro:

  - Hay dos canales LEDC independientes en el ESP32-S3, uno por motor.
    CH_DRIVE alimenta el pin ENB del puente H, que controla la
    velocidad del motor de traccion (las llantas traseras). CH_STEER
    alimenta el pin ENA del puente H, que controla cuanta fuerza tiene
    el motor de direccion (las llantas delanteras).

  - El sentido de giro NO lo da el PWM: lo dan los pines digitales
    IN1/IN2 (direccion) e IN3/IN4 (traccion) del puente H. El PWM solo
    decide la "fuerza", el puente H decide el "lado". Por ejemplo,
    para avanzar: IN3=HIGH, IN4=LOW, PWM en ENB con duty 200. Para
    retroceder: IN3=LOW, IN4=HIGH, PWM en ENB con duty 200.

  - La velocidad real del carro NO es exactamente proporcional al duty.
    A duty muy bajo el motor zumba pero las llantas no giran porque
    no logra vencer la friccion estatica del tren motriz. Por eso el
    firmware hace un truco que se llama "kick-start": al entrar a
    cada estado de movimiento aplica un PWM alto (kickStartPWM = 240)
    durante un tiempo corto (kickStartMs = 150 ms) para arrancar las
    llantas, y luego baja al PWM de crucero (parkDriveSpeed = 200).
    Asi nos aseguramos de que el carro siempre arranque, sin importar
    a que velocidad de crucero queramos ir despues.

  - Lo mismo aplica al motor de direccion. Como no es servo, no
    sabemos su posicion real: el firmware estima donde estan las
    llantas cronometrando cuanto tiempo lleva el motor moviendose
    en cada sentido, asumiendo que a steerSpeed PWM tarda tSteerFullMs
    en ir de tope a tope. Por eso al inicio el carro hace una
    calibracion (steerCalibrateHome) chocando contra el tope izquierdo
    para tener un cero conocido.



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


#Sliders del GUI (que hace cada uno)
Todos los sliders se mandan en vivo con "SET:clave=valor". El
firmware aplica los cambios al instante sin reiniciar.

Velocidades y direccion:
  - driveSpeed (PWM 0-255): Velocidad del motor de traccion en
    modo manual (cuando aprietas W o S). Default 180.
  - parkDriveSpeed (PWM 0-255): Velocidad del motor de traccion
    durante la maniobra automatica de estacionamiento. Tiene que
    ser suficiente para vencer la friccion estatica. Default 200.
  - steerSpeed (PWM 0-255): Fuerza con la que se mueve el motor de
    direccion. Mas alto = giro mas rapido pero menos preciso.
    Default 170.
  - tSteerFullMs (100-1500 ms): Tiempo que tarda la direccion en
    ir de tope izquierdo a tope derecho a steerSpeed PWM. Es la
    base para estimar la posicion de las llantas. Default 400.
  - steerTrimMs (-150 a +150 ms): Sesgo del centro de la direccion.
    Si las llantas no quedan derechas cuando el firmware piensa
    que estan al centro, este slider las corrige. Default 0.

Deteccion de huecos:
  - distCarroCm (5-100 cm): Si el sensor lateral lee MENOS que este
    valor, el firmware considera que esta viendo un carro estacionado.
    Default 25.
  - distHuecoCm (10-200 cm): Si el sensor lateral lee MAS que este
    valor, el firmware considera que ve un hueco. La banda entre
    distCarroCm y distHuecoCm es zona muerta para evitar oscilaciones.
    Siempre debe estar al menos 10 cm arriba de distCarroCm; si lo
    pones mal, el firmware lo corrige solo. Default 45.
  - distFrenaSuaveCm (3-50 cm): Cuando el sensor trasero lee menos
    de este valor durante REVERSA_GIRO, el carro frena suave para
    pasar a enderezar. Default 15.
  - distParaYaCm (2-30 cm): Cuando el sensor trasero lee menos de
    distParaYaCm + margenBanquetaCm durante REVERSA_RECTA, el carro
    se detiene definitivamente. Default 8.

Tiempos override (avanzados):
  - tAvanceInicialMs (0-10000 ms): Override manual del tiempo del
    avance inicial. Si esta en 0, se calcula a partir de
    distInicialCm y cmPorSegPark. Solo subirlo si quieres forzar un
    valor distinto al derivado. Default 0.
  - tAvanzarHuecoMs (0-5000 ms): Override manual del tiempo que el
    carro avanza despues de confirmar un hueco. Si esta en 0, se
    deriva de carLargoCm y cmPorSegPark. Default 0.
  - minHuecoStableMs (0-2000 ms): Tiempo minimo de estabilidad del
    hueco para confirmarlo. La logica nueva ya valida por longitud
    real, este parametro queda como override historico. Default 250.

Geometria del carro (la usa la maniobra para calcular distancias):
  - carLargoCm (5-100 cm): Largo fisico del carro. Default 29.
  - carAnchoCm (5-50 cm): Ancho fisico del carro. La rutina pide
    que el hueco mida al menos carAnchoCm + margenHuecoCm. Default 14.
  - carAltoCm (5-50 cm): Alto del carro. Solo informativo. Default 25.
  - margenHuecoCm (0-30 cm): Colchon de seguridad lateral. Se suma
    a carAnchoCm para definir el hueco minimo aceptable. Subirlo si
    el carro queda muy pegado a los vecinos. Default 6.
  - margenBanquetaCm (0-20 cm): Colchon de seguridad trasero. Se
    suma a distParaYaCm para frenar antes de tocar la banqueta.
    Subirlo si el carro pega con la pared trasera. Default 4.

Calibracion del modo automatico:
  - cmPorSegPark (5-200 cm/s): Velocidad real estimada del carro
    cuando se mueve a parkDriveSpeed. Es el parametro clave para
    convertir tiempos en distancias. Hay que medirlo a mano: con
    una regla en el suelo, mandar W exactamente 1 segundo y medir
    cuanto avanzo. Default 30.
  - distInicialCm (20-300 cm): Cuanto debe avanzar el carro al
    arrancar antes de empezar a buscar huecos. El reglamento del
    TMR pide 1 m. Default 100.
  - kickStartPWM (100-255): PWM alto que se aplica al inicio de
    cada estado de movimiento para vencer la friccion estatica del
    motor parado. Default 240.
  - kickStartMs (0-500 ms): Cuanto tiempo se mantiene el kickStartPWM
    antes de bajar al PWM de crucero. Default 150.
  - tReversaGiroMs (200-5000 ms): Duracion maxima de la fase de
    reversa con direccion al tope. Determina que tan profundo entra
    el carro en diagonal antes de enderezar. Hay que afinarlo en la
    pista segun el radio de giro real. Default 1600.


#Carpeta sketch_jan26a
Carpeta vieja con codigo de Arduino que recibia ordenes por puerto
serial. No se usa en la version actual, queda solo como referencia.


#contro_car.py
Version inicial del control en Python, no se usa.
