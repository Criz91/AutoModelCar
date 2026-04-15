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
  `SET:nombre:valor`.

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
Los mas importantes:

- `cmPorSegPark`: cuanto avanza el carro por segundo a `parkDriveSpeed`.
  De este valor depende toda la estimacion de distancias.
- `tReversaGiroMs`: cuanto tiempo de reversa con volante girado.
- `margenBanquetaCm`: cuanto se aleja del trasero antes de enderezar.
- `distHuecoCm`: minimo de hueco para considerarlo estacionable.

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
