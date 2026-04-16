"""
AutoModelCar - Cerebro autonomo para Raspberry Pi 5 + chip Hailo
TMR 2026 - Categoria AutoModelCar

Este script corre dentro del carro, montado sobre la Raspberry Pi 5.
Toma video con la picamera2, segmenta el carril con la red lane.hef
acelerada por el chip Hailo, y manda comandos al ESP32-S3 por UART.

Comandos enviados al ESP (texto plano, terminado en '\n'):
    W   avanzar
    A   girar a la izquierda
    D   girar a la derecha
    X   detener
    LF  entrar a modo line-follow local del ESP (curva cerrada)
    NOLF salir de modo line-follow y volver a control desde la Pi

Logica de handoff Hailo <-> Line Follow:
    - Si las 5 zonas verticales del mask suman menos que UMBRAL_ACTIVIDAD
      durante BLACKOUT_FRAMES seguidos, asumimos que la IA perdio las
      lineas (curva cerrada) y mandamos LF al ESP. Mientras estamos en LF
      la Pi NO manda W/A/D/X, el ESP se controla solo con los TCRT5000.
    - Cuando volvemos a ver carril con histeresis (suma > UMBRAL * 1.5)
      durante RECOVER_FRAMES seguidos, mandamos NOLF y retomamos control.

El servidor FastAPI en puerto 5000 expone un MJPEG con visualizacion de
zonas y comando actual, util para debug remoto desde la laptop por WiFi.

Uso:
    python3 master_autonomo_hailo.py           # modo normal
    python3 master_autonomo_hailo.py --test    # prueba UART y camara sin Hailo
"""

import os
import sys
import signal
import threading
import time
import traceback

# Verificar dependencias antes de importar para dar mensajes claros
_FALTAN = []
try:
    import cv2
except ImportError:
    _FALTAN.append("opencv-python (sudo apt install python3-opencv)")

try:
    import numpy as np
except ImportError:
    _FALTAN.append("numpy (pip install numpy)")

try:
    import serial
except ImportError:
    _FALTAN.append("pyserial (sudo apt install python3-serial)")

try:
    import uvicorn
    from fastapi import FastAPI
    from starlette.responses import StreamingResponse
except ImportError:
    _FALTAN.append("fastapi + uvicorn (pip install fastapi uvicorn)")

if _FALTAN:
    print("ERROR: faltan dependencias para correr el cerebro:")
    for d in _FALTAN:
        print("  -", d)
    print("Instala todo y vuelve a correr.")
    sys.exit(1)


# Configuracion principal
MODELO_LANE = "lane.hef"
BAUD_RATE = 115200
PUERTOS_UART_CANDIDATOS = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]

IMG_SIZE = 640            # tamano de entrada de la red
FRECUENCIA_ENVIO = 3      # procesar N frames antes de tomar decision
UMBRAL_ACTIVIDAD = 0.5    # minimo de actividad (suma escalada de zonas)

BLACKOUT_FRAMES = 3       # frames consecutivos sin carril -> entra a LF
RECOVER_FRAMES = 4        # frames consecutivos con carril -> sale de LF
HISTERESIS = 1.5          # multiplicador del umbral para recuperarse


class CarroCerebro:
    """Estado compartido entre el hilo de IA y el servidor FastAPI."""

    def __init__(self):
        self.frame_web = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ultimo_comando = ""
        self.modo_LF = False
        self.telemetria_esp = ""
        self.error_fatal = None   # si un hilo truena, guarda el mensaje


cerebro = CarroCerebro()


def abrir_uart():
    """Intenta abrir el puerto UART en los candidatos comunes de la Pi 5."""
    for puerto in PUERTOS_UART_CANDIDATOS:
        try:
            s = serial.Serial(puerto, BAUD_RATE, timeout=0.05)
            print("[OK] UART abierto en", puerto)
            return s
        except Exception as e:
            print("[--] UART", puerto, "no disponible:", e)
    print("[ERROR] No se pudo abrir ningun puerto UART.")
    print("        Verifica que el puerto serie este habilitado en raspi-config")
    print("        y que el cable UART Pi<->ESP este conectado.")
    return None


def enviar_comando(cmd, uart):
    """Envia un comando al ESP si el puerto esta disponible."""
    if uart is None:
        return
    try:
        uart.write((cmd + "\n").encode("ascii"))
        uart.flush()
    except Exception as e:
        print("[ERROR] Enviando", cmd, ":", e)


def loop_lectura_telemetria(uart):
    """Hilo que escucha lineas del ESP y guarda la ultima telemetria."""
    if uart is None:
        return
    buf = ""
    while not cerebro.stop_event.is_set():
        try:
            data = uart.read(128)
            if not data:
                continue
            buf += data.decode("ascii", errors="ignore")
            while "\n" in buf:
                linea, buf = buf.split("\n", 1)
                linea = linea.strip()
                if linea.startswith("{"):
                    cerebro.telemetria_esp = linea
        except Exception:
            time.sleep(0.05)


def loop_inteligencia(uart):
    """Hilo principal: captura, infiere, decide y publica visualizacion."""
    try:
        _loop_inteligencia_inner(uart)
    except Exception as e:
        msg = "HILO DE IA MURIO: " + str(e)
        print("[ERROR]", msg)
        traceback.print_exc()
        cerebro.error_fatal = msg
        # Seguridad: parar motores si la IA truena
        enviar_comando("X", uart)
        enviar_comando("NOLF", uart)


def _loop_inteligencia_inner(uart):
    """Logica real del hilo de IA, separada para manejar excepciones arriba."""

    # Importar picamera2 y hailo aqui para que el error sea claro
    try:
        from picamera2 import Picamera2
    except ImportError:
        raise RuntimeError(
            "No se pudo importar picamera2. "
            "Instala con: sudo apt install python3-picamera2"
        )

    try:
        from hailo_platform import (
            HEF,
            VDevice,
            InputVStreamParams,
            OutputVStreamParams,
            InferVStreams,
        )
    except ImportError:
        raise RuntimeError(
            "No se pudo importar hailo_platform. "
            "Instala el SDK de Hailo segun la documentacion oficial."
        )

    # Buscar el modelo HEF
    script_dir = os.path.dirname(os.path.abspath(__file__))
    modelo_path = os.path.join(script_dir, MODELO_LANE)
    if not os.path.exists(modelo_path):
        raise RuntimeError(
            "No se encontro el modelo '{}'. "
            "Coloca el archivo lane.hef en la misma carpeta que este script ({}).".format(
                MODELO_LANE, script_dir
            )
        )

    # Camara
    print("[..] Iniciando picamera2...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration({"size": (640, 480)}))
    picam2.start()
    print("[OK] Camara lista.")

    # Hailo
    print("[..] Cargando modelo Hailo:", modelo_path)
    target = VDevice()
    hef = HEF(modelo_path)
    network_group = target.configure(hef)[0]
    input_name = hef.get_input_vstream_infos()[0].name
    print("[OK] Hailo listo. Input:", input_name)

    contador_cuadros = 0
    blackout_cnt = 0
    recover_cnt = 0
    frames_total = 0
    lf_start_time = 0       # timestamp de cuando entro a LF
    LF_TIMEOUT_S = 10       # si lleva 10s en LF sin recuperar, salir

    with network_group.activate():
        params = [
            InputVStreamParams.make(network_group),
            OutputVStreamParams.make(network_group),
        ]
        with InferVStreams(network_group, *params) as pipe:
            print("[OK] Cerebro activo. Analizando carril a", IMG_SIZE, "x", IMG_SIZE)

            while not cerebro.stop_event.is_set():
                # Captura y pre-procesado
                frame_raw = picam2.capture_array()
                contador_cuadros += 1
                frames_total += 1

                if frame_raw.shape[-1] == 4:
                    img_rgb = cv2.cvtColor(frame_raw, cv2.COLOR_RGBA2RGB)
                else:
                    img_rgb = frame_raw

                img_resized = cv2.resize(img_rgb, (IMG_SIZE, IMG_SIZE))
                input_data = {
                    input_name: np.expand_dims(img_resized, axis=0).astype(np.uint8)
                }

                # Inferencia
                out = pipe.infer(input_data)
                mask = list(out.values())[0][0].astype(np.float32)

                # Segmentacion en 5 rebanadas verticales
                h, w = mask.shape[:2]
                seccion = w // 5
                factor = 10000
                z1 = np.sum(mask[:, :seccion]) / factor
                z2 = np.sum(mask[:, seccion:seccion * 2]) / factor
                z3 = np.sum(mask[:, seccion * 2:seccion * 3]) / factor
                z4 = np.sum(mask[:, seccion * 3:seccion * 4]) / factor
                z5 = np.sum(mask[:, seccion * 4:]) / factor
                actividad_total = z1 + z2 + z3 + z4 + z5

                # Diagnostico: imprimir estado cada ~30 frames (~1 seg)
                if frames_total % 30 == 0:
                    print(
                        "[IA] frame={} act={:.2f} cmd={} LF={} bo={} rc={}".format(
                            frames_total, actividad_total,
                            cerebro.ultimo_comando or "-",
                            cerebro.modo_LF, blackout_cnt, recover_cnt
                        )
                    )

                # Deteccion de blackout / recovery
                if actividad_total < UMBRAL_ACTIVIDAD:
                    blackout_cnt += 1
                    recover_cnt = 0
                    if blackout_cnt >= BLACKOUT_FRAMES and not cerebro.modo_LF:
                        enviar_comando("LF", uart)
                        cerebro.modo_LF = True
                        cerebro.ultimo_comando = "LF"
                        lf_start_time = time.time()
                        print("[LF] Blackout detectado, entrando a line-follow")
                else:
                    blackout_cnt = 0
                    if cerebro.modo_LF:
                        if actividad_total > UMBRAL_ACTIVIDAD * HISTERESIS:
                            recover_cnt += 1
                            if recover_cnt >= RECOVER_FRAMES:
                                enviar_comando("NOLF", uart)
                                cerebro.modo_LF = False
                                cerebro.ultimo_comando = ""
                                recover_cnt = 0
                                lf_start_time = 0
                                print("[LF] Carril recuperado, volviendo a Hailo")
                        else:
                            recover_cnt = 0

                # Timeout LF: si lleva mucho tiempo en LF sin recuperar,
                # forzar salida para no quedarse atorado
                if cerebro.modo_LF and lf_start_time > 0:
                    if (time.time() - lf_start_time) > LF_TIMEOUT_S:
                        enviar_comando("NOLF", uart)
                        cerebro.modo_LF = False
                        cerebro.ultimo_comando = ""
                        lf_start_time = 0
                        blackout_cnt = 0
                        print("[LF] Timeout {}s, forzando salida de line-follow".format(
                            LF_TIMEOUT_S))

                # Toma de decision normal (solo si NO estamos en LF)
                if (not cerebro.modo_LF) and contador_cuadros >= FRECUENCIA_ENVIO:
                    comando_decidido = "W"

                    if z3 > z2 and z3 > z4 and z3 > UMBRAL_ACTIVIDAD:
                        comando_decidido = "W"
                    elif (z1 + z2) > (z4 + z5) and (z1 + z2) > UMBRAL_ACTIVIDAD:
                        comando_decidido = "A"
                    elif (z4 + z5) > (z1 + z2) and (z4 + z5) > UMBRAL_ACTIVIDAD:
                        comando_decidido = "D"
                    elif actividad_total < UMBRAL_ACTIVIDAD:
                        comando_decidido = "X"

                    if comando_decidido != cerebro.ultimo_comando:
                        enviar_comando(comando_decidido, uart)
                        cerebro.ultimo_comando = comando_decidido

                    contador_cuadros = 0

                # Visualizacion para el monitor web
                frame_viz = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                for i in range(1, 5):
                    cv2.line(
                        frame_viz,
                        (i * (640 // 5), 0),
                        (i * (640 // 5), 480),
                        (255, 255, 0),
                        1,
                    )

                color_orden = (0, 255, 0) if not cerebro.modo_LF else (0, 165, 255)
                cv2.putText(
                    frame_viz,
                    "ORDEN: " + cerebro.ultimo_comando,
                    (20, 60),
                    cv2.FONT_HERSHEY_PLAIN,
                    3,
                    color_orden,
                    4,
                )
                cv2.putText(
                    frame_viz,
                    "Z3: {:.1f}".format(z3),
                    (240, 450),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.5,
                    (255, 255, 255),
                    2,
                )

                _, buffer = cv2.imencode(
                    ".jpg", frame_viz, [cv2.IMWRITE_JPEG_QUALITY, 50]
                )
                with cerebro.lock:
                    cerebro.frame_web = buffer.tobytes()

    picam2.stop()


def test_uart_y_camara():
    """Modo de prueba: verifica UART y camara sin necesitar Hailo."""
    print("=== MODO DE PRUEBA ===")
    print()

    # 1. UART
    uart = abrir_uart()
    if uart is None:
        print("UART: FALLO. Verifica raspi-config y el cableado.")
    else:
        print("UART: OK. Mandando PING al ESP...")
        enviar_comando("PING", uart)
        time.sleep(0.5)
        respuesta = ""
        while uart.in_waiting:
            respuesta += uart.read(uart.in_waiting).decode("ascii", errors="ignore")
        if "PONG" in respuesta:
            print("UART: El ESP respondio PONG. Comunicacion OK.")
        else:
            print("UART: No llego PONG. Respuesta:", repr(respuesta))
            print("       Verifica que el ESP tenga el firmware cargado")
            print("       y que TX de la Pi va a RX del ESP (y viceversa).")
        print()

        # Probar mover el carro brevemente
        print("UART: Mandando W durante 0.5 s para probar traccion...")
        enviar_comando("W", uart)
        time.sleep(0.5)
        enviar_comando("X", uart)
        print("UART: Si el carro avanzo y freno, la traccion funciona.")
        print()

    # 2. Camara
    try:
        from picamera2 import Picamera2
        print("Picamera2: importado OK.")
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration({"size": (640, 480)}))
        picam2.start()
        frame = picam2.capture_array()
        print("Picamera2: captura OK. Shape:", frame.shape)
        picam2.stop()
    except ImportError:
        print("Picamera2: NO INSTALADO. sudo apt install python3-picamera2")
    except Exception as e:
        print("Picamera2: ERROR:", e)
    print()

    # 3. Hailo
    try:
        from hailo_platform import HEF
        print("Hailo: importado OK.")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        modelo_path = os.path.join(script_dir, MODELO_LANE)
        if os.path.exists(modelo_path):
            print("Hailo: modelo", MODELO_LANE, "encontrado.")
        else:
            print("Hailo: FALTA el modelo", MODELO_LANE)
            print("       Coloca lane.hef en:", script_dir)
    except ImportError:
        print("Hailo: NO INSTALADO. Instala el SDK oficial de Hailo.")
    print()

    print("=== FIN DE PRUEBA ===")
    if uart:
        uart.close()


# Servidor FastAPI - monitoreo remoto del video por WiFi
app = FastAPI()


@app.get("/")
def video_feed():
    def generate():
        while True:
            with cerebro.lock:
                if cerebro.frame_web:
                    yield (
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                        + cerebro.frame_web
                        + b"\r\n"
                    )
            time.sleep(0.04)

    return StreamingResponse(
        generate(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


@app.get("/telemetria")
def telemetria():
    return {
        "esp": cerebro.telemetria_esp,
        "modo_LF": cerebro.modo_LF,
        "error": cerebro.error_fatal,
    }


@app.get("/status")
def status():
    """Endpoint rapido para verificar que el script esta vivo."""
    return {
        "vivo": True,
        "ultimo_comando": cerebro.ultimo_comando,
        "modo_LF": cerebro.modo_LF,
        "error": cerebro.error_fatal,
    }


def shutdown(signum, frame):
    """Stop graceful: detener motores y salir de LF antes de cerrar."""
    print("Deteniendo cerebro...")
    cerebro.stop_event.set()
    if _uart_global is not None:
        enviar_comando("X", _uart_global)
        enviar_comando("NOLF", _uart_global)
    time.sleep(0.2)
    sys.exit(0)


_uart_global = None


def test_drive():
    """Modo --test-drive: manda una secuencia de comandos por UART sin
    necesitar Hailo, camara ni seguidores de linea. Solo prueba que la
    comunicacion Pi -> UART -> ESP funciona y que el carro se mueve.

    Uso: python3 master_autonomo_hailo.py --test-drive

    Mientras corre, mira el log de la GUI: deben aparecer lineas
    RX,UART,W / RX,UART,A / etc. Si no aparecen, el UART no esta
    conectado o los cables TX/RX estan invertidos.
    """
    print("=== TEST DRIVE (sin IA, sin camara, sin seguidores) ===")
    print()
    uart = abrir_uart()
    if uart is None:
        print("No se pudo abrir UART. No se puede probar.")
        sys.exit(1)

    print("Mandando PING al ESP...")
    enviar_comando("PING", uart)
    time.sleep(0.5)
    respuesta = ""
    while uart.in_waiting:
        respuesta += uart.read(uart.in_waiting).decode("ascii", errors="ignore")
    if "PONG" in respuesta:
        print("[OK] ESP respondio PONG. UART funciona.")
    else:
        print("[!!] No llego PONG. Respuesta:", repr(respuesta))
        print("     Verifica cables y que el ESP tenga firmware cargado.")
        print("     Continuo con la prueba de todas formas...")
    print()

    pasos = [
        ("W",  1.5, "Avanzar recto 1.5 seg"),
        ("X",  0.5, "Frenar"),
        ("A",  0.4, "Girar izquierda 0.4 seg"),
        ("C",  0.3, "Centrar direccion"),
        ("W",  1.0, "Avanzar 1 seg"),
        ("X",  0.5, "Frenar"),
        ("D",  0.4, "Girar derecha 0.4 seg"),
        ("C",  0.3, "Centrar direccion"),
        ("S",  1.0, "Reversa 1 seg"),
        ("X",  0.5, "Frenar"),
    ]

    print("Secuencia de prueba ({} pasos):".format(len(pasos)))
    for cmd, dur, desc in pasos:
        print("  {} -> {} ({:.1f}s)".format(cmd, desc, dur))
    print()

    input("Presiona ENTER para empezar (el carro se va a mover)...")
    print()

    for i, (cmd, dur, desc) in enumerate(pasos):
        print("[{}/{}] {} -> {}".format(i + 1, len(pasos), cmd, desc))
        enviar_comando(cmd, uart)
        time.sleep(dur)

    enviar_comando("X", uart)
    print()
    print("[OK] Secuencia terminada.")
    print("     Si el carro se movio, la comunicacion Pi->ESP funciona.")
    print("     Si no se movio, revisa:")
    print("       - Que GPIO14(TX) de la Pi va a GPIO16(RX) del ESP")
    print("       - Que GPIO15(RX) de la Pi va a GPIO17(TX) del ESP")
    print("       - Que comparten GND")
    uart.close()


if __name__ == "__main__":
    # Modo test: verifica hardware sin Hailo
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        test_uart_y_camara()
        sys.exit(0)

    # Modo test-drive: prueba UART mandando W/A/D/S sin IA
    if len(sys.argv) > 1 and sys.argv[1] == "--test-drive":
        test_drive()
        sys.exit(0)

    print("AutoModelCar - Cerebro autonomo TMR 2026")
    print()

    # Abrir UART
    _uart_global = abrir_uart()
    if _uart_global is None:
        print("Sin UART el carro no se va a mover.")
        print("Si solo quieres probar el video, continuo de todas formas...")
        print()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    t_ia = threading.Thread(target=loop_inteligencia, args=(_uart_global,), daemon=True)
    t_ia.start()

    t_tel = threading.Thread(target=loop_lectura_telemetria, args=(_uart_global,), daemon=True)
    t_tel.start()

    # Esperar a que el hilo de IA arranque o falle. Hailo puede tardar
    # varios segundos en cargar el modelo, asi que damos 10 segundos.
    print("[..] Esperando a que el hilo de IA arranque (hasta 10s)...")
    time.sleep(10)
    if cerebro.error_fatal:
        print()
        print("El hilo de IA no pudo arrancar:")
        print(" ", cerebro.error_fatal)
        print()
        print("Corrige el error y vuelve a correr.")
        print("Tip: usa 'python3 master_autonomo_hailo.py --test'")
        print("     para diagnosticar pieza por pieza.")
        sys.exit(1)

    print()
    print("Monitor web activo en http://[IP_DE_LA_PI]:5000")
    print("  /          -> video MJPEG")
    print("  /status    -> estado del cerebro")
    print("  /telemetria -> ultima telemetria del ESP")
    print()
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="error")
