"""
AutoModelCar - Cerebro autonomo para Raspberry Pi 5 + chip Hailo
TMR 2026 - Categoria AutoModelCar

Este script corre dentro del carro, montado sobre la Raspberry Pi 5.
Toma video con la picamera2, segmenta el carril con la red lane.hef
acelerada por el chip Hailo, y manda comandos al ESP32-S3 por UART.
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


# ============================================================
# CONFIGURACION PRINCIPAL
# ============================================================
MODELO_LANE = "lane.hef"
MODELO_DETECT = "detect-supremo.hef"

BAUD_RATE = 115200
PUERTOS_UART_CANDIDATOS = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]

IMG_SIZE = 640  # tamano de entrada de la red lane

# ---------------- Velocidades ----------------
# Recta, curva suave y curva fuerte
VELOCIDAD_AUTONOMA_RECTA = 180
VELOCIDAD_AUTONOMA_CURVA = 145
VELOCIDAD_AUTONOMA_CURVA_FUERTE = 125

# Velocidad usada cuando se detecta un stop y se va aproximando
VELOCIDAD_APROX = 140

# ---------------- Decisiones de carril ----------------
GRID_FILAS = 4
GRID_COLS = 5
PESOS_FILAS = [0.1, 0.3, 0.6, 1.0]

UMBRAL_ACTIVIDAD = 0.05

# Umbrales de error
UMBRAL_ERROR_RECTA = 0.16
UMBRAL_ERROR_CURVA = 0.30
UMBRAL_ERROR_CURVA_FUERTE = 0.48

# Suavizado del cambio de estado
CONFIRMACIONES_CAMBIO = 1

BLACKOUT_FRAMES = 5
RECOVER_FRAMES = 4
HISTERESIS = 1.5

# ---------------- Keep-alive y direccion por pulsos ----------------
# El ESP32 corta por timeout si no recibe comandos. Se manda W periodicamente.
PERIODO_KEEPALIVE_W_S = 0.30

# Cada cuanto tiempo se puede volver a mandar un pulso de direccion
PERIODO_PULSO_DIRECCION_S = 0.14

# Cuanto dura el pulso A/D antes de mandar C
DURACION_PULSO_DIRECCION_S = 0.06

# ---------------- Deteccion de senales ----------------
DETECT_CLASE_STOP = None
DETECT_NOMBRE_STOP = "stop"
DETECT_CONF_MIN = 0.50

DETECT_AREA_APROX = 0.04
DETECT_AREA_STOP = 0.10

STOP_ESPERA_S = 10
STOP_COOLDOWN_S = 20
DETECT_FEED_INTERVALO = 6


# ============================================================
# ESTADO GLOBAL
# ============================================================
class CarroCerebro:
    """Estado compartido entre IA, deteccion y servidor web."""

    def __init__(self):
        self.frame_web = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.uart_write_lock = threading.Lock()

        self.ultimo_comando = ""
        self.modo_LF = False
        self.telemetria_esp = ""
        self.error_fatal = None

        # Deteccion de senales
        self.frame_detect = None
        self.frame_detect_lock = threading.Lock()
        self.frame_detect_event = threading.Event()
        self.parada_activa = False
        self.stop_state = "normal"      # normal | aproximando | detenido | cooldown
        self.stop_state_since = 0.0

        # Control autonomo mas robusto
        self.ultimo_drive_cmd = ""
        self.ultimo_steer_cmd = ""
        self.ultima_velocidad_enviada = None

        self.keepalive_w_last = 0.0
        self.steer_last_pulse = 0.0
        self.steer_pulse_until = 0.0

        self.autonomia_activa = False
        self.velocidad_actual_objetivo = VELOCIDAD_AUTONOMA_RECTA
        self.direccion_objetivo = "C"   # A / D / C


cerebro = CarroCerebro()


# ============================================================
# UART
# ============================================================
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
    """Envia un comando ASCII terminado en \\n al ESP32."""
    if uart is None:
        return
    try:
        with cerebro.uart_write_lock:
            uart.write((cmd + "\n").encode("ascii"))
            uart.flush()
    except Exception as e:
        print("[ERROR] Enviando", cmd, ":", e)


def loop_lectura_telemetria(uart):
    """Hilo que escucha lineas del ESP y guarda la ultima telemetria JSON."""
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


# ============================================================
# DETECCION DE SENALES
# ============================================================
def _parse_detecciones(output_dict, img_h, img_w, primer_frame):
    """
    Intenta parsear la salida de detect-supremo.hef y devuelve una lista de
    (clase_id_o_nombre, confianza, area_relativa) para cada deteccion valida.
    """
    detecciones = []

    for key, tensor in output_dict.items():
        arr = np.squeeze(tensor)

        if primer_frame:
            print("[DETECT-INFO] key='{}' shape={} dtype={}".format(
                key, arr.shape, arr.dtype))

        # Caso 1: [N, 6] o [N, 5+C]
        if arr.ndim == 2 and arr.shape[-1] >= 6:
            for row in arr:
                conf = float(row[4])
                if conf < DETECT_CONF_MIN:
                    continue

                class_val = row[5] if arr.shape[-1] == 6 else int(np.argmax(row[5:]))
                class_id = int(class_val) if arr.shape[-1] == 6 else class_val

                if arr.shape[-1] >= 6:
                    x1, y1, x2, y2 = float(row[0]), float(row[1]), float(row[2]), float(row[3])
                    if x2 <= 1.0 and y2 <= 1.0:
                        area = abs((x2 - x1) * (y2 - y1))
                    else:
                        area = abs((x2 - x1) * (y2 - y1)) / (img_w * img_h)
                else:
                    area = 0.0

                if primer_frame:
                    print("[DETECT-INFO]   clase={} conf={:.2f} area={:.3f}".format(
                        class_id, conf, area))

                detecciones.append((class_id, conf, area))

        # Caso 2: YOLOv5 [N, 85]
        elif arr.ndim == 2 and arr.shape[-1] == 85:
            for row in arr:
                obj_conf = float(row[4])
                if obj_conf < DETECT_CONF_MIN * 0.5:
                    continue

                class_id = int(np.argmax(row[5:]))
                conf = obj_conf * float(row[5 + class_id])
                if conf < DETECT_CONF_MIN:
                    continue

                x_c, y_c, bw, bh = row[0], row[1], row[2], row[3]
                area = float(bw * bh)
                if x_c > 1.0:
                    area /= (img_w * img_h)

                if primer_frame:
                    print("[DETECT-INFO]   clase={} conf={:.2f} area={:.3f}".format(
                        class_id, conf, area))

                detecciones.append((class_id, conf, area))

    return detecciones


def _es_stop(clase_id):
    """Devuelve True si la clase corresponde a stop sign."""
    if DETECT_CLASE_STOP is None:
        return False
    return int(clase_id) == int(DETECT_CLASE_STOP)


def _loop_deteccion_inner(uart):
    """Logica del hilo de deteccion de señales."""
    try:
        from hailo_platform import HEF, VDevice, InputVStreamParams, OutputVStreamParams, InferVStreams
    except ImportError:
        print("[DETECT] hailo_platform no disponible. Hilo de deteccion desactivado.")
        return

    script_dir = os.path.dirname(os.path.abspath(__file__))
    detect_path = os.path.join(script_dir, MODELO_DETECT)

    if not os.path.exists(detect_path):
        print("[DETECT] No se encontro '{}'. Hilo de deteccion desactivado.".format(detect_path))
        print("[DETECT] Coloca detect-supremo.hef en:", script_dir)
        return

    print("[DETECT] Cargando modelo:", detect_path)
    try:
        target = VDevice()
        hef = HEF(detect_path)
        ng = target.configure(hef)[0]
        in_name = hef.get_input_vstream_infos()[0].name
        print("[DETECT] Modelo listo. Input:", in_name)
    except Exception as e:
        print("[DETECT] Error al cargar modelo:", e)
        return

    primer_frame = True
    stop_cooldown_end = 0.0

    with ng.activate():
        params_in = InputVStreamParams.make(ng)
        params_out = OutputVStreamParams.make(ng)
        with InferVStreams(ng, params_in, params_out) as pipe:
            while not cerebro.stop_event.is_set():
                got_frame = cerebro.frame_detect_event.wait(timeout=0.5)
                cerebro.frame_detect_event.clear()

                if not got_frame:
                    continue

                with cerebro.frame_detect_lock:
                    frame_rgb = cerebro.frame_detect

                if frame_rgb is None:
                    continue

                img_h, img_w = frame_rgb.shape[:2]
                try:
                    det_in_info = hef.get_input_vstream_infos()[0]
                    det_h = det_in_info.shape[1] if len(det_in_info.shape) >= 2 else 640
                    det_w = det_in_info.shape[2] if len(det_in_info.shape) >= 3 else 640
                except Exception:
                    det_h, det_w = 640, 640

                img_resized = cv2.resize(frame_rgb, (det_w, det_h))
                input_data = {
                    in_name: np.expand_dims(img_resized, axis=0).astype(np.uint8)
                }

                try:
                    output = pipe.infer(input_data)
                except Exception as e:
                    print("[DETECT] Error en inferencia:", e)
                    continue

                dets = _parse_detecciones(output, img_h, img_w, primer_frame)
                primer_frame = False

                ahora = time.time()

                if ahora < stop_cooldown_end:
                    continue

                mejor_area = 0.0
                for (clase_id, conf, area) in dets:
                    if _es_stop(clase_id) and area > mejor_area:
                        mejor_area = area

                estado_actual = cerebro.stop_state

                if estado_actual == "normal":
                    if mejor_area >= DETECT_AREA_APROX:
                        cerebro.stop_state = "aproximando"
                        cerebro.stop_state_since = ahora
                        enviar_comando("HAZON", uart)
                        enviar_comando("SET:driveSpeed={}".format(VELOCIDAD_APROX), uart)
                        cerebro.ultima_velocidad_enviada = VELOCIDAD_APROX
                        print("[STOP] Aproximando al stop (area={:.3f})".format(mejor_area))

                elif estado_actual == "aproximando":
                    if mejor_area >= DETECT_AREA_STOP:
                        cerebro.parada_activa = True
                        cerebro.stop_state = "detenido"
                        cerebro.stop_state_since = ahora
                        enviar_comando("X", uart)
                        print("[STOP] Detenido en stop (area={:.3f}). Esperando {}s".format(
                            mejor_area, STOP_ESPERA_S))
                    elif mejor_area < DETECT_AREA_APROX * 0.5:
                        cerebro.stop_state = "normal"
                        enviar_comando("HAZOFF", uart)
                        enviar_comando("SET:driveSpeed={}".format(VELOCIDAD_AUTONOMA_RECTA), uart)
                        cerebro.ultima_velocidad_enviada = VELOCIDAD_AUTONOMA_RECTA
                        print("[STOP] Stop perdido, volviendo a normal")

                elif estado_actual == "detenido":
                    if (ahora - cerebro.stop_state_since) >= STOP_ESPERA_S:
                        cerebro.stop_state = "cooldown"
                        cerebro.parada_activa = False
                        enviar_comando("HAZOFF", uart)
                        enviar_comando("SET:driveSpeed={}".format(VELOCIDAD_AUTONOMA_RECTA), uart)
                        cerebro.ultima_velocidad_enviada = VELOCIDAD_AUTONOMA_RECTA
                        stop_cooldown_end = ahora + STOP_COOLDOWN_S
                        print("[STOP] Reanudando circuito (cooldown {}s)".format(STOP_COOLDOWN_S))

                elif estado_actual == "cooldown":
                    if ahora >= stop_cooldown_end:
                        cerebro.stop_state = "normal"
                        print("[STOP] Cooldown terminado, deteccion activa de nuevo")


def loop_deteccion(uart):
    """Wrapper con manejo de excepciones para el hilo de deteccion."""
    try:
        _loop_deteccion_inner(uart)
    except Exception as e:
        print("[DETECT] Hilo de deteccion murio:", e)
        traceback.print_exc()


# ============================================================
# CONTROL AUTONOMO HACIA EL ESP32
# ============================================================
def set_velocidad_objetivo(speed, uart):
    """Solo manda SET:driveSpeed cuando el valor cambia."""
    speed = int(max(0, min(255, speed)))
    if cerebro.ultima_velocidad_enviada != speed:
        enviar_comando("SET:driveSpeed={}".format(speed), uart)
        cerebro.ultima_velocidad_enviada = speed


def iniciar_pulso_direccion(cmd, uart):
    """
    Manda un pulso corto A/D y programa el auto-centrado con C.
    cmd debe ser 'A' o 'D'.
    """
    ahora = time.time()

    if cmd not in ("A", "D"):
        return

    if (ahora - cerebro.steer_last_pulse) < PERIODO_PULSO_DIRECCION_S:
        return

    enviar_comando(cmd, uart)
    cerebro.ultimo_comando = cmd
    cerebro.ultimo_steer_cmd = cmd
    cerebro.steer_last_pulse = ahora
    cerebro.steer_pulse_until = ahora + DURACION_PULSO_DIRECCION_S


def actualizar_direccion_objetivo(direccion_objetivo, uart):
    """
    direccion_objetivo:
      - 'A' -> pulso izquierda
      - 'D' -> pulso derecha
      - 'C' -> centrar/parar direccion
    """
    ahora = time.time()

    # Si habia un pulso activo y ya vencio, mandar C
    if cerebro.steer_pulse_until > 0 and ahora >= cerebro.steer_pulse_until:
        enviar_comando("C", uart)
        cerebro.ultimo_steer_cmd = "C"
        cerebro.steer_pulse_until = 0.0
        if cerebro.ultimo_comando in ("A", "D"):
            cerebro.ultimo_comando = "C"

    if direccion_objetivo == "C":
        if cerebro.ultimo_steer_cmd != "C" and cerebro.steer_pulse_until == 0.0:
            enviar_comando("C", uart)
            cerebro.ultimo_steer_cmd = "C"
            cerebro.ultimo_comando = "C"
        return

    if direccion_objetivo in ("A", "D"):
        iniciar_pulso_direccion(direccion_objetivo, uart)


def mantener_traccion(uart):
    """
    Manda W cada cierto tiempo como latido.
    Esto evita timeout y mantiene el carro avanzando aunque haya correcciones.
    """
    ahora = time.time()
    if (ahora - cerebro.keepalive_w_last) >= PERIODO_KEEPALIVE_W_S:
        enviar_comando("W", uart)
        cerebro.keepalive_w_last = ahora
        cerebro.ultimo_drive_cmd = "W"
        cerebro.ultimo_comando = "W"


def detener_control_autonomo(uart, reason=""):
    """Frena y centra al salir de autonomia."""
    cerebro.autonomia_activa = False
    cerebro.direccion_objetivo = "C"
    cerebro.steer_pulse_until = 0.0

    enviar_comando("C", uart)
    enviar_comando("X", uart)

    cerebro.ultimo_drive_cmd = "X"
    cerebro.ultimo_steer_cmd = "C"
    cerebro.ultimo_comando = "X"

    if reason:
        print("[AUTO] Control autonomo detenido:", reason)


# ============================================================
# IA PRINCIPAL
# ============================================================
def loop_inteligencia(uart):
    """Hilo principal: captura, infiere, decide y publica visualizacion."""
    try:
        _loop_inteligencia_inner(uart)
    except Exception as e:
        msg = "HILO DE IA MURIO: " + str(e)
        print("[ERROR]", msg)
        traceback.print_exc()
        cerebro.error_fatal = msg
        enviar_comando("C", uart)
        enviar_comando("X", uart)
        enviar_comando("NOLF", uart)


def _loop_inteligencia_inner(uart):
    """Logica real del hilo de IA."""

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

    script_dir = os.path.dirname(os.path.abspath(__file__))
    modelo_path = os.path.join(script_dir, MODELO_LANE)
    if not os.path.exists(modelo_path):
        raise RuntimeError(
            "No se encontro el modelo '{}'. "
            "Coloca el archivo lane.hef en la misma carpeta que este script ({}).".format(
                MODELO_LANE, script_dir
            )
        )

    print("[..] Iniciando picamera2...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration({"size": (640, 480)}))
    picam2.start()
    print("[OK] Camara lista.")

    print("[..] Cargando modelo Hailo:", modelo_path)
    target = VDevice()
    hef = HEF(modelo_path)
    network_group = target.configure(hef)[0]
    input_name = hef.get_input_vstream_infos()[0].name
    print("[OK] Hailo listo. Input:", input_name)

    set_velocidad_objetivo(VELOCIDAD_AUTONOMA_RECTA, uart)
    print("[OK] Velocidad autonoma enviada: driveSpeed={}".format(VELOCIDAD_AUTONOMA_RECTA))

    blackout_cnt = 0
    recover_cnt = 0
    frames_total = 0
    lf_start_time = 0.0
    LF_TIMEOUT_S = 10

    cmd_candidato_prev = ""
    confirmaciones = 0

    with network_group.activate():
        params = [
            InputVStreamParams.make(network_group),
            OutputVStreamParams.make(network_group),
        ]

        with InferVStreams(network_group, *params) as pipe:
            print("[OK] Cerebro activo. Analizando carril a", IMG_SIZE, "x", IMG_SIZE)

            while not cerebro.stop_event.is_set():
                frame_raw = picam2.capture_array()
                frames_total += 1

                if frame_raw.shape[-1] == 4:
                    img_rgb = cv2.cvtColor(frame_raw, cv2.COLOR_RGBA2RGB)
                else:
                    img_rgb = frame_raw

                img_resized = cv2.resize(img_rgb, (IMG_SIZE, IMG_SIZE))
                input_data = {
                    input_name: np.expand_dims(img_resized, axis=0).astype(np.uint8)
                }

                out = pipe.infer(input_data)
                mask = list(out.values())[0][0].astype(np.float32)

                h, w = mask.shape[:2]
                row_h = h // GRID_FILAS
                col_w = w // GRID_COLS
                col_pos = np.linspace(-1.0, 1.0, GRID_COLS)

                numerador = 0.0
                denominador = 0.0
                actividad_total = 0.0

                for r in range(GRID_FILAS):
                    peso_r = PESOS_FILAS[r]
                    r0 = r * row_h
                    r1 = (r + 1) * row_h if r < GRID_FILAS - 1 else h

                    for c in range(GRID_COLS):
                        c0 = c * col_w
                        c1 = (c + 1) * col_w if c < GRID_COLS - 1 else w

                        suma_celda = float(np.sum(mask[r0:r1, c0:c1]))
                        actividad_total += suma_celda

                        celda_ponderada = suma_celda * peso_r
                        numerador += celda_ponderada * col_pos[c]
                        denominador += celda_ponderada

                actividad_norm = actividad_total / (h * w) if h * w > 0 else 0.0

                if denominador > 0.0:
                    centroid_error = float(np.clip(numerador / denominador, -1.0, 1.0))
                else:
                    centroid_error = 0.0

                if frames_total % 30 == 0:
                    print(
                        "[IA] frame={} act={:.3f} err={:+.2f} cmd={} LF={} stop={} bo={} rc={} vel={}".format(
                            frames_total,
                            actividad_norm,
                            centroid_error,
                            cerebro.ultimo_comando or "-",
                            cerebro.modo_LF,
                            cerebro.stop_state,
                            blackout_cnt,
                            recover_cnt,
                            cerebro.ultima_velocidad_enviada,
                        )
                    )

                # --------------------------------------------------
                # Fallback a line-follow
                # --------------------------------------------------
                if actividad_norm < UMBRAL_ACTIVIDAD:
                    blackout_cnt += 1
                    recover_cnt = 0
                    if blackout_cnt >= BLACKOUT_FRAMES and not cerebro.modo_LF:
                        detener_control_autonomo(uart, reason="blackout->LF")
                        enviar_comando("LF", uart)
                        cerebro.modo_LF = True
                        cerebro.ultimo_comando = "LF"
                        lf_start_time = time.time()
                        print("[LF] Blackout detectado, entrando a line-follow")
                else:
                    blackout_cnt = 0
                    if cerebro.modo_LF:
                        if actividad_norm > UMBRAL_ACTIVIDAD * HISTERESIS:
                            recover_cnt += 1
                            if recover_cnt >= RECOVER_FRAMES:
                                enviar_comando("NOLF", uart)
                                cerebro.modo_LF = False
                                cerebro.ultimo_comando = ""
                                recover_cnt = 0
                                lf_start_time = 0.0
                                print("[LF] Carril recuperado, volviendo a Hailo")
                        else:
                            recover_cnt = 0

                if cerebro.modo_LF and lf_start_time > 0:
                    if (time.time() - lf_start_time) > LF_TIMEOUT_S:
                        enviar_comando("NOLF", uart)
                        cerebro.modo_LF = False
                        cerebro.ultimo_comando = ""
                        lf_start_time = 0.0
                        blackout_cnt = 0
                        print("[LF] Timeout {}s, forzando salida de line-follow".format(
                            LF_TIMEOUT_S))

                # --------------------------------------------------
                # Alimentar hilo de deteccion
                # --------------------------------------------------
                if frames_total % DETECT_FEED_INTERVALO == 0:
                    with cerebro.frame_detect_lock:
                        cerebro.frame_detect = img_rgb.copy()
                    cerebro.frame_detect_event.set()

                # --------------------------------------------------
                # CONTROL AUTONOMO REAL
                # --------------------------------------------------
                if not cerebro.modo_LF and not cerebro.parada_activa:
                    error_abs = abs(centroid_error)

                    if centroid_error < -UMBRAL_ERROR_CURVA_FUERTE:
                        nuevo_cmd = "A"
                        velocidad_obj = VELOCIDAD_AUTONOMA_CURVA_FUERTE
                    elif centroid_error > UMBRAL_ERROR_CURVA_FUERTE:
                        nuevo_cmd = "D"
                        velocidad_obj = VELOCIDAD_AUTONOMA_CURVA_FUERTE
                    elif centroid_error < -UMBRAL_ERROR_CURVA:
                        nuevo_cmd = "A"
                        velocidad_obj = VELOCIDAD_AUTONOMA_CURVA
                    elif centroid_error > UMBRAL_ERROR_CURVA:
                        nuevo_cmd = "D"
                        velocidad_obj = VELOCIDAD_AUTONOMA_CURVA
                    elif centroid_error < -UMBRAL_ERROR_RECTA:
                        nuevo_cmd = "A"
                        velocidad_obj = VELOCIDAD_AUTONOMA_CURVA
                    elif centroid_error > UMBRAL_ERROR_RECTA:
                        nuevo_cmd = "D"
                        velocidad_obj = VELOCIDAD_AUTONOMA_CURVA
                    else:
                        nuevo_cmd = "C"
                        velocidad_obj = VELOCIDAD_AUTONOMA_RECTA

                    # Suavizado
                    if nuevo_cmd == cmd_candidato_prev:
                        confirmaciones += 1
                    else:
                        confirmaciones = 1
                        cmd_candidato_prev = nuevo_cmd

                    if confirmaciones >= CONFIRMACIONES_CAMBIO:
                        cerebro.autonomia_activa = True
                        cerebro.direccion_objetivo = nuevo_cmd
                        cerebro.velocidad_actual_objetivo = velocidad_obj

                        set_velocidad_objetivo(velocidad_obj, uart)

                        # Mantener avance SIEMPRE
                        mantener_traccion(uart)

                        # Corregir direccion por pulsos
                        actualizar_direccion_objetivo(nuevo_cmd, uart)

                        if nuevo_cmd == "C":
                            cerebro.ultimo_comando = "W"
                        else:
                            cerebro.ultimo_comando = "W+" + nuevo_cmd

                        confirmaciones = CONFIRMACIONES_CAMBIO

                else:
                    detener_control_autonomo(uart, reason="LF o parada_activa")

                # --------------------------------------------------
                # VIZ
                # --------------------------------------------------
                frame_viz = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

                for r in range(1, GRID_FILAS):
                    y = r * (480 // GRID_FILAS)
                    cv2.line(frame_viz, (0, y), (640, y), (80, 80, 80), 1)

                for c in range(1, GRID_COLS):
                    x = c * (640 // GRID_COLS)
                    cv2.line(frame_viz, (x, 0), (x, 480), (255, 255, 0), 1)

                cx = int((centroid_error + 1.0) / 2.0 * 640)
                cv2.line(frame_viz, (cx, 0), (cx, 480), (0, 255, 0), 2)
                cv2.line(frame_viz, (320, 0), (320, 480), (0, 0, 200), 1)

                color_orden = (0, 255, 0) if not cerebro.modo_LF else (0, 165, 255)

                cv2.putText(
                    frame_viz,
                    "ORDEN: " + str(cerebro.ultimo_comando),
                    (20, 60),
                    cv2.FONT_HERSHEY_PLAIN,
                    3,
                    color_orden,
                    4,
                )
                cv2.putText(
                    frame_viz,
                    "ERR:{:+.2f} ACT:{:.3f} SPD:{} STOP:{}".format(
                        centroid_error,
                        actividad_norm,
                        cerebro.ultima_velocidad_enviada,
                        cerebro.stop_state
                    ),
                    (10, 450),
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


# ============================================================
# PRUEBAS
# ============================================================
def test_uart_y_camara():
    print("=== MODO DE PRUEBA ===")
    print()

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

        print("UART: Mandando W durante 0.5 s para probar traccion...")
        enviar_comando("W", uart)
        time.sleep(0.5)
        enviar_comando("X", uart)
        print("UART: Si el carro avanzo y freno, la traccion funciona.")
        print()

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


def test_drive():
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
        ("SET:driveSpeed={}".format(VELOCIDAD_AUTONOMA_RECTA), 0.2, "Configurar velocidad"),
        ("W", 1.5, "Avanzar recto 1.5 seg"),
        ("X", 0.5, "Frenar"),
        ("A", 0.08, "Pulso izquierda"),
        ("C", 0.20, "Centrar direccion"),
        ("W", 1.0, "Avanzar 1 seg"),
        ("X", 0.5, "Frenar"),
        ("D", 0.08, "Pulso derecha"),
        ("C", 0.20, "Centrar direccion"),
        ("W", 1.0, "Avanzar 1 seg"),
        ("X", 0.5, "Frenar"),
    ]

    print("Secuencia de prueba ({} pasos):".format(len(pasos)))
    for cmd, dur, desc in pasos:
        print("  {} -> {} ({:.2f}s)".format(cmd, desc, dur))
    print()

    input("Presiona ENTER para empezar (el carro se va a mover)...")
    print()

    for i, (cmd, dur, desc) in enumerate(pasos):
        print("[{}/{}] {} -> {}".format(i + 1, len(pasos), cmd, desc))
        enviar_comando(cmd, uart)
        time.sleep(dur)

    enviar_comando("X", uart)
    enviar_comando("C", uart)

    print()
    print("[OK] Secuencia terminada.")
    print("     Si el carro se movio, la comunicacion Pi->ESP funciona.")
    print("     Si no se movio, revisa:")
    print("       - Que GPIO14(TX) de la Pi va a GPIO16(RX) del ESP")
    print("       - Que GPIO15(RX) de la Pi va a GPIO17(TX) del ESP")
    print("       - Que comparten GND")
    uart.close()


# ============================================================
# FASTAPI
# ============================================================
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
        "stop_state": cerebro.stop_state,
        "velocidad_actual": cerebro.ultima_velocidad_enviada,
        "ultimo_drive_cmd": cerebro.ultimo_drive_cmd,
        "ultimo_steer_cmd": cerebro.ultimo_steer_cmd,
    }


@app.get("/status")
def status():
    return {
        "vivo": True,
        "ultimo_comando": cerebro.ultimo_comando,
        "modo_LF": cerebro.modo_LF,
        "error": cerebro.error_fatal,
    }


# ============================================================
# SHUTDOWN
# ============================================================
def shutdown(signum, frame):
    print("Deteniendo cerebro...")
    cerebro.stop_event.set()
    if _uart_global is not None:
        enviar_comando("C", _uart_global)
        enviar_comando("X", _uart_global)
        enviar_comando("NOLF", _uart_global)
    time.sleep(0.2)
    sys.exit(0)


_uart_global = None


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        test_uart_y_camara()
        sys.exit(0)

    if len(sys.argv) > 1 and sys.argv[1] == "--test-drive":
        test_drive()
        sys.exit(0)

    print("AutoModelCar - Cerebro autonomo TMR 2026")
    print()

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

    t_det = threading.Thread(target=loop_deteccion, args=(_uart_global,), daemon=True)
    t_det.start()

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
    print("  /           -> video MJPEG")
    print("  /status     -> estado del cerebro")
    print("  /telemetria -> ultima telemetria del ESP")
    print()

    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="error")