# AutoModelCar - TMR 2026

Codigo unificado del carro autonomo del equipo para la categoria
**AutoModelCar** del Torneo Mexicano de Robotica 2026.

El sistema combina tres piezas:

- **ESP32-S3** con firmware unificado que controla motores, sensores
  ultrasonicos, seguidores de linea, intermitentes y la maquina de
  estados del estacionamiento en bateria.
- **Laptop** con una GUI de Python para teleoperar, calibrar y disparar
  el estacionamiento por WiFi.
- **Raspberry Pi 5 + chip Hailo** montada sobre el carro, que detecta el
  carril con una red neuronal y manda comandos al ESP por UART.

---

## Arquitectura

```
       Laptop                              ESP32-S3
   +-----------+    WiFi  TCP 8080    +---------------+
   | control_  | <------------------> | firmware      |
   | gui.py    |   AutoModelCar_TMR26 | ESP_MAIN.ino  |
   +-----------+                      |               |
                                      |               | --> motores L298N
   Raspberry Pi 5 (a bordo)           |               | --> 4 ultrasonicos
   +-----------+    UART 115200       |               | --> 3 TCRT5000
   | master_   | <------------------> |               | --> LEDs hazard
   | autonomo_ |  pines RX=16 TX=17   +---------------+
   | hailo.py  |
   +-----------+
        |
        v
   picamera2 + Hailo (lane.hef)
   FastAPI MJPEG en puerto 5000
```

Reglas relevantes del reglamento TMR 2026:

- 2.7 prohibe comunicacion **cableada** con procesamiento externo. La
  Pi va montada en el carro, asi que UART entre Pi y ESP es comunicacion
  interna del robot, lo cual esta permitido. La GUI de la laptop SOLO se
  conecta por WiFi.
- 3.3 obliga a que el carro se pueda teleoperar. Por eso mantenemos el
  servidor TCP en el ESP aunque el modo principal sea autonomo.

---

## Estructura del repo

```
firmware/ESP_MAIN/ESP_MAIN.ino   firmware unificado para el ESP32-S3
control_gui.py                   GUI Tkinter, cliente TCP, corre en laptop
rpi/master_autonomo_hailo.py     cerebro autonomo, corre en la Raspberry Pi 5
requirements.txt                 dependencias minimas de la GUI
README.md                        este archivo
```

---

## 1. Que cargar en el ESP32-S3

**Archivo:** `firmware/ESP_MAIN/ESP_MAIN.ino`

**Placa en Arduino IDE:** `ESP32S3 Dev Module` (o el modelo concreto de la
placa, p.ej. `ESP32-S3-WROOM-1`). Configurar:

- Flash size: 16 MB
- PSRAM: OPI PSRAM
- Partition scheme: el default

**Librerias necesarias** (Library Manager):

- `ArduinoJson` (v6.x)
- `WiFi` (incluida con el core de ESP32)

**Pasos:**

1. Abrir Arduino IDE.
2. File -> Open -> seleccionar `firmware/ESP_MAIN/ESP_MAIN.ino`.
3. Tools -> Board -> ESP32 Arduino -> ESP32S3 Dev Module.
4. Tools -> Port -> el puerto USB del ESP.
5. Sketch -> Upload.

Una vez flasheado, el ESP arranca un AP WiFi:

- **SSID:** `AutoModelCar_TMR26`
- **Password:** `tmr2026robot`
- **IP del ESP:** `192.168.4.1`
- **Puerto TCP:** `8080`

### Pinout del firmware

| Funcion              | Pin(es)                       |
|----------------------|-------------------------------|
| Direccion (motor)    | IN1=12, IN2=11, ENA=8         |
| Traccion (motor)     | IN3=14, IN4=13, ENB=18        |
| Ultrasonico derecho  | TRIG=4,  ECHO=5               |
| Ultrasonico izq      | TRIG=6,  ECHO=7               |
| Ultrasonico trasero  | TRIG=15, ECHO=9               |
| Ultrasonico frontal  | TRIG=1,  ECHO=2               |
| TCRT5000 izquierda   | 38                            |
| TCRT5000 derecha     | 39                            |
| TCRT5000 centro      | 40                            |
| LED hazard izq       | 10                            |
| LED hazard der       | 21                            |
| UART hacia Pi        | RX=16, TX=17 (HardwareSerial2)|

> Nota: los TCRT5000 con comparador entregan LOW cuando ven la linea
> blanca (reflectiva). El firmware lo asume con `LINE_ACTIVE_LOW = 1`.
> Si en tu modulo es al reves, cambia esa macro y recompila.

---

## 2. Que correr en la laptop (GUI)

**Archivo:** `control_gui.py`

**Requisitos:**

```
python -m venv venv
venv\Scripts\activate         # en Windows
pip install -r requirements.txt
```

(`requirements.txt` solo trae `pynput` y `six`. `tkinter` y `socket` son
de la libreria estandar de Python, no se instalan.)

**Pasos para usar:**

1. Encender el carro. Esperar a que el ESP cree el AP `AutoModelCar_TMR26`.
2. En la laptop, conectarse a esa red WiFi (password `tmr2026robot`).
3. Correr `python control_gui.py`.
4. La GUI ya viene apuntando a `192.168.4.1:8080`. Click en **Connect**.
5. Verificar que la telemetria empieza a llegar (sensores R/L/B/F y
   estado MANUAL en verde).

**Controles principales:**

- **WASD**: teleop (W avance, S reversa, A izq, D der, X stop).
- **PR** / **PL**: dispara el estacionamiento en bateria a la derecha o
  izquierda. La maquina de estados corre 100% en el ESP.
- **LF** / **NOLF**: fuerza modo line-follow del ESP (util para probar
  los TCRT5000 sin la Pi).
- **H**: toggle de intermitentes.
- **Sliders**: 23 parametros editables en vivo (velocidad, distancias
  del estacionamiento, tiempos de giro, etc). Cada cambio se envia como
  `SET:nombre=valor`. Detalles abajo en la seccion *Sliders*.

---

## Como funciona el PWM (control de motores)

El carro tiene **dos motores DC** controlados por un puente H **L298N**:

- **Motor de traccion** (las dos llantas traseras): empuja el carro
  hacia adelante o reversa.
- **Motor de direccion** (la llanta delantera): es un motor DC simple,
  no un servo. Para girar el volante hay que aplicarle voltaje durante
  un cierto tiempo (`tSteerFullMs`) y para regresar al centro hay que
  aplicar el voltaje al reves durante el mismo tiempo. Por eso el
  firmware lleva una **estimacion de posicion** (`steerPosMs`) que
  cuenta cuanto tiempo y en que direccion ha girado.

### Que es PWM

PWM (Pulse Width Modulation, modulacion por ancho de pulso) es una
tecnica para simular voltajes intermedios usando una salida que solo
puede estar en HIGH (3.3 V) o LOW (0 V).

En lugar de dar al motor el voltaje "a medias", el ESP enciende y apaga
el pin a alta velocidad. El **% de tiempo encendido** se llama **duty
cycle** y es lo que determina la velocidad efectiva del motor:

```
duty = 0   (0/255)   --> motor parado
duty = 128 (128/255) --> motor a media potencia
duty = 255 (255/255) --> motor a maxima potencia
```

El L298N recibe ese PWM en sus pines de **enable** (`ENA` para
direccion, `ENB` para traccion) y los pines `IN1/IN2` y `IN3/IN4`
deciden el **sentido** de giro (HIGH/LOW invierte la polaridad).

### Configuracion del PWM en el firmware

```c
const int PWM_FREQ = 20000;   // 20 kHz - inaudible
const int PWM_RES  = 8;       // resolucion 8 bits -> 0..255
const int CH_STEER = 0;       // canal LEDC para direccion (ENA)
const int CH_DRIVE = 1;       // canal LEDC para traccion  (ENB)
```

- **20 kHz** se eligio para que el motor no chille (el oido humano deja
  de escuchar arriba de ~18 kHz).
- **8 bits** da 256 niveles de velocidad, suficiente para este carro.
- El ESP32 usa el periferico **LEDC** que tiene varios canales por
  hardware, y aqui usamos el canal 0 para la direccion y el canal 1
  para la traccion.

### Kick-start (vencer la friccion estatica)

Un motor DC parado tiene mas friccion que uno ya girando. Si le mandas
de golpe `driveSpeed = 180`, puede que no arranque y se quede zumbando.

Para evitarlo, cada vez que el carro pasa de parado a moverse el
firmware aplica un **pulso fuerte** durante unos pocos ms:

```
millis() 0 ............. kickStartMs ............. infinito
PWM      kickStartPWM    -->                       cruise (driveSpeed)
```

Esto se hace en `armKickStart()` y `driveForwardKick()` /
`driveReverseKick()`. Los dos parametros son sliders:

- `kickStartPWM` (default 240): que tan fuerte es el pulso inicial.
- `kickStartMs` (default 150): cuanto dura ese pulso.

Si el carro se queda atorado al arrancar, sube `kickStartPWM` o
`kickStartMs`. Si arranca con un brinco brusco, bajalos.

---

## Como funcionan los sliders (flujo end-to-end)

Cuando mueves un slider en la GUI pasa esto:

```
1. Tu mueves el slider en control_gui.py
        |
        v
2. Tkinter dispara el callback con el nuevo valor (ej. 195)
        |
        v
3. self.send("SET:driveSpeed=195")
        |
        v
4. socket.sendall(b"SET:driveSpeed=195\n")  --> WiFi TCP
        |
        v
5. ESP32 recibe la linea por su servidor TCP en el puerto 8080
        |
        v
6. handleCommand() detecta que empieza con "SET:" y parsea
        |
        v
7. applyParam("driveSpeed", 195)
        |
        v
8. P.driveSpeed = constrain(195, 0, 255)
        |
        v
9. La proxima vez que el firmware llame a setMotor(...) usa el nuevo
   valor de P.driveSpeed automaticamente.
```

Puntos clave:

- **No hay que reflashear** el ESP para cambiar parametros. Todo se
  ajusta en vivo desde la GUI.
- **`constrain(v, min, max)`** en `applyParam()` (ver firmware linea
  749) limita los valores para evitar que metas algo absurdo
  (ej. velocidad negativa o tiempos imposibles).
- **Persistencia**: los valores **NO se guardan** en flash. Si reinicias
  el ESP, vuelve a los defaults del struct `Params P` (firmware linea
  66). Si encontras una calibracion buena, anotala y luego copia los
  valores como nuevos defaults antes de la competencia.
- **Telemetria**: el ESP no devuelve ack del SET, pero si miras la GUI
  veras los valores actualizados porque la propia GUI los recuerda.

---

## Sliders - que hace cada uno

### Velocidades (PWM 0..255)

| Slider             | Default | Que hace                                                                 |
|--------------------|---------|--------------------------------------------------------------------------|
| `driveSpeed`       | 180     | PWM de traccion en modo MANUAL (W/S). Mas alto = mas rapido.             |
| `parkDriveSpeed`   | 200     | PWM de traccion durante el estacionamiento. Suele ser mayor que el manual porque las maniobras necesitan mas torque para arrancar de parado. |
| `steerSpeed`       | 170     | PWM aplicado al motor de direccion mientras gira. No afecta cuanto gira, solo que tan rapido llega al tope. |
| `lineFollowSpeed`  | 150     | PWM de traccion durante modo `LINE_FOLLOW`. Va mas lento que `driveSpeed` porque las correcciones con TCRT5000 son mas bruscas. |
| `kickStartPWM`     | 240     | PWM del pulso inicial para vencer friccion al arrancar. Ver seccion PWM. |
| `kickStartMs`      | 150     | Duracion de ese pulso inicial en milisegundos.                           |

### Direccion (volante)

| Slider          | Default | Que hace                                                                              |
|-----------------|---------|---------------------------------------------------------------------------------------|
| `tSteerFullMs`  | 400     | Tiempo en ms que tarda el volante en ir de un extremo al otro. Es la "regla" para estimar la posicion del volante. Si tu motor de direccion es mas lento, sube este valor. |
| `steerTrimMs`   | 0       | Correccion fina del centro. Si el carro tira solo a la izquierda yendo recto, sube unos +20 ms; si tira a la derecha, baja a -20 ms. Es el "trim" del volante. |

### Distancias para el estacionamiento (cm)

| Slider              | Default | Que hace                                                                                |
|---------------------|---------|-----------------------------------------------------------------------------------------|
| `distInicialCm`     | 100     | Distancia inicial (en cm) que avanza el carro antes de empezar a buscar hueco. Coincide con el "1 m" de la regla. |
| `distCarroCm`       | 25      | Cuantos cm consecutivos de objeto sigue contando el ultrasonico lateral antes de declarar "hay un coche". |
| `distHuecoCm`       | 45      | Cuantos cm consecutivos de hueco hacen falta para declarar "este es el espacio". Tipicamente debe ser `carLargoCm + margenHuecoCm` o mas. El firmware lo fuerza a ser al menos `distCarroCm + 10`. |
| `distFrenaSuaveCm`  | 15      | Cuando el ultrasonico **trasero** lee menos que esto durante la reversa, el carro empieza a frenar suave. |
| `distParaYaCm`      | 8       | Cuando el trasero lee menos que esto, paro absoluto. Margen de seguridad contra la banqueta. |
| `margenHuecoCm`     | 6       | Holgura extra que se exige al hueco encontrado, ademas del ancho del carro. |
| `margenBanquetaCm`  | 4       | Distancia minima que se quiere dejar entre la cola del carro y la banqueta al final. |
| `cmPorSegPark`      | 30      | **Calibracion critica**. Cuantos cm avanza el carro por segundo cuando va a `parkDriveSpeed`. De este valor depende toda la conversion tiempo<->distancia durante el parking. Como medirlo: ver seccion *Calibracion*. |

### Tiempos del estacionamiento (ms)

| Slider              | Default | Que hace                                                                                |
|---------------------|---------|-----------------------------------------------------------------------------------------|
| `tAvanceInicialMs`  | 0       | Si > 0, fuerza a usar este tiempo en lugar de calcular `distInicialCm / cmPorSegPark`. Util para saltarse la fase inicial cuando pruebas en banco. |
| `tAvanzarHuecoMs`   | 0       | Tiempo extra de avance una vez detectado el hueco, para alinear el eje trasero con el inicio del cajon. 0 = calculado automaticamente. |
| `minHuecoStableMs`  | 250     | Cuanto tiempo el ultrasonico lateral debe leer "hueco" continuo antes de aceptarlo. Filtra falsos positivos por ruido. |
| `tReversaGiroMs`    | 1600    | Cuanto tiempo el carro va en reversa con el volante girado a tope antes de enderezarlo. Es **el slider mas critico** para que el carro entre derecho. Mucho = se mete demasiado torcido; poco = no alcanza a meter la cola. |
| `maxAutoMs`         | 60000   | Timeout global del estacionamiento. Si pasan 60 s sin terminar, aborta por seguridad.   |

### Geometria del carro (cm)

| Slider          | Default | Que hace                                                                              |
|-----------------|---------|---------------------------------------------------------------------------------------|
| `carLargoCm`    | 29      | Largo fisico del carro. Se usa para calcular el hueco minimo exigido y los avances internos del parking. **Medilo** con cinta despues de armar todo. |
| `carAnchoCm`    | 14      | Ancho fisico. Se usa con `margenHuecoCm` para validar el hueco lateral.               |
| `carAltoCm`     | 25      | Alto fisico. Solo informativo (para el reglamento de dimensiones), no afecta logica.  |

---

## Calibracion paso a paso

Antes de la primera competencia conviene calibrar, en orden:

1. **`steerTrimMs`**: en una superficie plana, manda `W` y observa.
   Si el carro tira a un lado, ajusta `steerTrimMs` hasta que vaya
   recto.
2. **`tSteerFullMs`**: con el carro al aire, manda `A` y mide con
   cronometro cuanto tarda el volante en ir de tope a tope. Mete ese
   valor (en ms).
3. **`kickStartPWM` / `kickStartMs`**: en piso, baja `driveSpeed` a
   ~140. Si el carro no arranca, sube `kickStartPWM` hasta que arranque
   sin brinco.
4. **`cmPorSegPark`**: pon el carro en el piso, cronometro en mano,
   manda `W` durante exactamente 2 segundos y mide cuantos cm avanzo.
   Divide entre 2. Mete ese valor. (Tip: marca el piso con cinta.)
5. **Geometria**: mide largo, ancho y alto reales del carro y mete los
   valores en `carLargoCm`, `carAnchoCm`, `carAltoCm`.
6. **Estacionamiento real**: prueba `PR` con un cajon real. Si el carro
   se queda corto al meterse, sube `tReversaGiroMs`. Si se mete
   demasiado torcido, bajalo. Si choca con la banqueta, sube
   `margenBanquetaCm` o `distParaYaCm`.

Anota los valores finales y, opcionalmente, copialos como nuevos
defaults en `firmware/ESP_MAIN/ESP_MAIN.ino` (struct `Params P`,
linea 66) antes de la competencia. Asi quedan persistentes.

---

## 3. Que correr en la Raspberry Pi 5

**Archivo:** `rpi/master_autonomo_hailo.py`

**Requisitos en la Pi:**

```
sudo apt install python3-picamera2 python3-opencv python3-serial
pip install fastapi uvicorn numpy
# Hailo platform: instalar el SDK oficial de Hailo segun su documentacion
```

Ademas necesitas el modelo entrenado **`lane.hef`** en el mismo directorio
que el script, o ajustar `MODELO_LANE` en el codigo.

**Habilitar UART en la Pi 5:**

1. `sudo raspi-config` -> Interface Options -> Serial Port.
2. "Login shell over serial?" -> No.
3. "Serial port hardware enabled?" -> Yes.
4. Reiniciar.

El script intenta abrir `/dev/serial0`, `/dev/ttyAMA0` y `/dev/ttyS0`
en ese orden; usa el primero que responda.

**Cableado UART Pi <-> ESP32-S3:**

| Pi (header)    | ESP32-S3   |
|----------------|------------|
| GPIO14 (TXD)   | GPIO16 (RX)|
| GPIO15 (RXD)   | GPIO17 (TX)|
| GND            | GND        |

**Como correr:**

```
python3 rpi/master_autonomo_hailo.py
```

La Pi:

- Captura video con la picamera2.
- Segmenta el carril con `lane.hef` en el chip Hailo.
- Decide W/A/D/X y lo envia al ESP por UART.
- Si pierde el carril por mas de 3 frames seguidos (curva cerrada),
  manda `LF` al ESP. El ESP entra a su loop local de TCRT5000 a 100 Hz
  y se conduce solo. Cuando la Pi recupera el carril (con histeresis),
  manda `NOLF` y retoma el control.
- Expone un MJPEG en `http://[IP_DE_LA_PI]:5000/` para ver el video con
  zonas y comando actual desde la laptop.

---

## Protocolo ESP <-> mundo

El firmware acepta comandos por **TCP**, **UART** y **Serial USB**, todos
en texto plano terminado en `\n`. La telemetria sale por los tres
canales como una linea JSON cada 200 ms.

### Comandos basicos

| Comando         | Efecto                                          |
|-----------------|-------------------------------------------------|
| `W` `S`         | avanzar / reversa                               |
| `A` `D`         | girar izq / der                                 |
| `X`             | stop                                            |
| `PR` `PL`       | iniciar estacionamiento bateria der / izq       |
| `LF` `NOLF`     | entrar / salir de modo line-follow local        |
| `H`             | toggle intermitentes (hazards)                  |
| `T`             | toggle SENSOR_TEST (dump cada 500 ms)           |
| `PING`          | el ESP responde con un `PONG`                   |
| `SET:<k>:<v>`   | actualiza un parametro (slider) en caliente     |

### Telemetria (JSON)

```
{"t":"tel","mode":"MANUAL","st":"IDLE",
 "R":42,"L":40,"B":99,"F":120,
 "lnL":0,"lnR":0,"lnC":1,
 "sp":200,"side":1}
```

Significado de cada campo:

- `mode`: `MANUAL`, `AUTO`, `LF`, `TEST`, `DONE`, `ABORT`.
- `st`: subestado del parking (`IDLE`, `MEDIR_HUECO`, `REV_GIRO`, ...).
- `R`/`L`/`B`/`F`: distancia en cm de los 4 ultrasonicos (filtrada con
  mediana de 5 muestras).
- `lnL`/`lnR`/`lnC`: 1 si el TCRT5000 ve la linea blanca, 0 si no.
- `sp`: posicion estimada de la direccion en ms desde el centro.
- `side`: lado del estacionamiento (+1 derecha, -1 izquierda).

---

## Estacionamiento en bateria - como funciona

La maquina de estados del ESP corre 9 estados en orden:

1. `AVANCE_INICIAL`     - avanza 1 m desde la posicion de salida.
2. `MEDIR_HUECO`        - va contando con el ultrasonico lateral hasta
                          encontrar un hueco estable mayor al carro.
3. `AVANZAR_PASA_HUECO` - se mete unos cm mas para alinear el eje.
4. `STOP_Y_GIRAR`       - frena y gira el volante a tope hacia el lado.
5. `REVERSA_GIRO`       - reversa con el volante girado, vigilando el
                          ultrasonico trasero.
6. `STOP_Y_ENDEREZAR`   - frena y endereza el volante.
7. `REVERSA_RECTA`      - termina de meterse recto.
8. `ESTACIONADO`        - apaga motores y enciende intermitentes.
9. `ABORTADO`           - si algo sale mal, frena y reporta error.

Los 23 parametros del slider permiten calibrar todo esto sin recompilar.
Ver la seccion **Sliders** mas arriba para la tabla completa de cada
uno y la seccion **Calibracion paso a paso** para el orden recomendado
de ajuste.

---

## Checklist de competencia

Antes de cada corrida:

- [ ] Baterias del carro y del router cargadas.
- [ ] SSID `AutoModelCar_TMR26` visible desde la laptop.
- [ ] Telemetria llegando a la GUI (4 ultrasonicos + 3 sensores de linea).
- [ ] Prueba de teleop WASD: el watchdog de 1500 ms detiene si sueltas
      la tecla.
- [ ] `PR` y `PL` completan los 9 estados con el carro al aire.
- [ ] Hazards encienden con `H`.
- [ ] Pi conectada al ESP por UART, video MJPEG visible en :5000.
- [ ] Tapar la camara con la mano: el ESP debe entrar a `LF` en menos
      de 200 ms y los TCRT5000 deben corregir solos. Destapar: vuelve a
      Hailo.
- [ ] Dimensiones del carro armado debajo de 45x35x45 cm.

---

## Troubleshooting rapido

- **La GUI no conecta**: verifica que estas en la red WiFi del ESP y no
  en otra. El ESP es AP, no cliente.
- **Telemetria llega pero los motores no se mueven**: revisa el
  watchdog (`GLOBAL_CMD_TIMEOUT_MS = 1500`). Si no recibe ningun
  comando en 1.5 s, frena por seguridad. Manda algo y vuelve a probar.
- **Estacionamiento aborta enseguida**: `maxAutoMs` se cumplio, o el
  ultrasonico trasero esta siempre por debajo de `margenBanquetaCm`.
  Levanta el carro y observa el flujo de estados en la GUI.
- **La Pi no abre UART**: corre `ls -l /dev/serial*` y verifica que
  exista; activa el puerto serie en `raspi-config`.
- **Hailo no carga `lane.hef`**: verifica que el archivo este en el
  mismo directorio del script, o edita `MODELO_LANE`.
