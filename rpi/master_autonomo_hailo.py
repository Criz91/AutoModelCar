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
"""

import cv2
import glob
import numpy as np
import signal
import sys
import threading
import time

import serial
import uvicorn
from fastapi import FastAPI
from starlette.responses import StreamingResponse

from picamera2 import Picamera2
from hailo_platform import (
    HEF,
    VDevice,
    InputVStreamParams,
    OutputVStreamParams,
    InferVStreams,
)


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
        self.telemetria_esp = ""  # ultima linea JSON recibida del ESP


cerebro = CarroCerebro()


def abrir_uart():
    """Intenta abrir el puerto UART en los candidatos comunes de la Pi 5."""
    for puerto in PUERTOS_UART_CANDIDATOS:
        try:
            s = serial.Serial(puerto, BAUD_RATE, timeout=0.05)
            print("UART abierto en", puerto)
            return s
        except Exception as e:
            print("UART", puerto, "no disponible:", e)
    print("ADVERTENCIA: no se pudo abrir ningun UART. El carro NO se movera.")
    return None


esp32 = abrir_uart()


def enviar_comando(cmd):
    """Envia un comando al ESP si el puerto esta disponible."""
    if esp32 is None:
        return
    try:
        esp32.write((cmd + "\n").encode("ascii"))
        esp32.flush()
    except Exception as e:
        print("Error enviando", cmd, ":", e)


def loop_lectura_telemetria():
    """Hilo que escucha lineas del ESP y guarda la ultima telemetria."""
    if esp32 is None:
        return
    buf = ""
    while not cerebro.stop_event.is_set():
        try:
            data = esp32.read(128)
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


def loop_inteligencia():
    """Hilo principal: captura, infiere, decide y publica visualizacion."""

    # Camara
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration({"size": (640, 480)}))
    picam2.start()

    # Hailo
    target = VDevice()
    hef = HEF(MODELO_LANE)
    network_group = target.configure(hef)[0]
    input_name = hef.get_input_vstream_infos()[0].name

    contador_cuadros = 0
    blackout_cnt = 0
    recover_cnt = 0

    with network_group.activate():
        params = [
            InputVStreamParams.make(network_group),
            OutputVStreamParams.make(network_group),
        ]
        with InferVStreams(network_group, *params) as pipe:
            print("Cerebro activo. Analizando carril a", IMG_SIZE, "x", IMG_SIZE)

            while not cerebro.stop_event.is_set():
                # Captura y pre-procesado
                frame_raw = picam2.capture_array()
                contador_cuadros += 1

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

                # Deteccion de blackout / recovery (siempre)
                if actividad_total < UMBRAL_ACTIVIDAD:
                    blackout_cnt += 1
                    recover_cnt = 0
                    if blackout_cnt >= BLACKOUT_FRAMES and not cerebro.modo_LF:
                        enviar_comando("LF")
                        cerebro.modo_LF = True
                        cerebro.ultimo_comando = "LF"
                else:
                    blackout_cnt = 0
                    if cerebro.modo_LF:
                        if actividad_total > UMBRAL_ACTIVIDAD * HISTERESIS:
                            recover_cnt += 1
                            if recover_cnt >= RECOVER_FRAMES:
                                enviar_comando("NOLF")
                                cerebro.modo_LF = False
                                cerebro.ultimo_comando = ""
                                recover_cnt = 0
                        else:
                            recover_cnt = 0

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
                        enviar_comando(comando_decidido)
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
    return {"esp": cerebro.telemetria_esp, "modo_LF": cerebro.modo_LF}


def shutdown(signum, frame):
    """Stop graceful: detener motores y salir de LF antes de cerrar."""
    print("Deteniendo cerebro...")
    cerebro.stop_event.set()
    enviar_comando("X")
    enviar_comando("NOLF")
    time.sleep(0.2)
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    t_ia = threading.Thread(target=loop_inteligencia, daemon=True)
    t_ia.start()

    t_tel = threading.Thread(target=loop_lectura_telemetria, daemon=True)
    t_tel.start()

    print("Monitor web activo en http://[IP_DE_LA_PI]:5000")
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="error")
