"""
AutoModelCar - Cerebro autonomo para Raspberry Pi 5 + chip Hailo
TMR 2026 - Categoria AutoModelCar
"""

import os
import sys
import signal
import threading
import time
import traceback

_FALTAN = []
try:
    import cv2
except ImportError:
    _FALTAN.append("opencv-python")
try:
    import numpy as np
except ImportError:
    _FALTAN.append("numpy")
try:
    import serial
except ImportError:
    _FALTAN.append("pyserial")
try:
    import uvicorn
    from fastapi import FastAPI
    from starlette.responses import StreamingResponse
except ImportError:
    _FALTAN.append("fastapi + uvicorn")

if _FALTAN:
    sys.exit(1)


MODELO_LANE = "lane.hef"
BAUD_RATE = 115200
PUERTOS_UART_CANDIDATOS = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]

IMG_SIZE = 640            

VELOCIDAD_AUTONOMA = 195

GRID_FILAS   = 4
GRID_COLS    = 5    

PESOS_FILAS  = [0.1, 0.3, 0.6, 1.0]   

UMBRAL_ACTIVIDAD = 0.05
UMBRAL_ERROR_GIRO = 0.30   
CONFIRMACIONES_CAMBIO = 1

BLACKOUT_FRAMES = 5       
RECOVER_FRAMES  = 4       
HISTERESIS      = 1.5     

MODELO_DETECT = "detect-supremo.hef"
DETECT_CLASE_STOP   = None   
DETECT_NOMBRE_STOP  = "stop" 
DETECT_CONF_MIN = 0.50

DETECT_AREA_APROX = 0.04   
DETECT_AREA_STOP  = 0.10   

VELOCIDAD_APROX = 150
STOP_ESPERA_S = 10 # Tiempo oficial de espera para señales y peatones
STOP_COOLDOWN_S = 20
DETECT_FEED_INTERVALO = 6

class CarroCerebro:
    def __init__(self):
        self.frame_web = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ultimo_comando = ""
        self.modo_LF = False
        self.telemetria_esp = ""
        self.error_fatal = None   

        self.frame_detect = None       
        self.frame_detect_lock = threading.Lock()
        self.frame_detect_event = threading.Event()  
        self.parada_activa = False     
        self.stop_state = "normal"     
        self.stop_state_since = 0.0    

cerebro = CarroCerebro()

def abrir_uart():
    for puerto in PUERTOS_UART_CANDIDATOS:
        try:
            s = serial.Serial(puerto, BAUD_RATE, timeout=0.05)
            return s
        except Exception:
            pass
    return None

def enviar_comando(cmd, uart):
    if uart is None: return
    try:
        uart.write((cmd + "\n").encode("ascii"))
        uart.flush()
    except Exception:
        pass

def loop_lectura_telemetria(uart):
    if uart is None: return
    buf = ""
    while not cerebro.stop_event.is_set():
        try:
            data = uart.read(128)
            if not data: continue
            buf += data.decode("ascii", errors="ignore")
            while "\n" in buf:
                linea, buf = buf.split("\n", 1)
                linea = linea.strip()
                if linea.startswith("{"): cerebro.telemetria_esp = linea
        except Exception:
            time.sleep(0.05)

def loop_inteligencia(uart):
    try:
        _loop_inteligencia_inner(uart)
    except Exception as e:
        cerebro.error_fatal = str(e)
        enviar_comando("X", uart)
        enviar_comando("NOLF", uart)

def _parse_detecciones(output_dict, img_h, img_w, primer_frame):
    detecciones = []
    for key, tensor in output_dict.items():
        arr = np.squeeze(tensor)   
        if arr.ndim == 2 and arr.shape[-1] >= 6:
            for row in arr:
                conf = float(row[4])
                if conf < DETECT_CONF_MIN: continue
                class_val = row[5] if arr.shape[-1] == 6 else int(np.argmax(row[5:]))
                class_id  = int(class_val) if arr.shape[-1] == 6 else class_val
                if arr.shape[-1] >= 6:
                    x1, y1, x2, y2 = float(row[0]), float(row[1]), float(row[2]), float(row[3])
                    if x2 <= 1.0 and y2 <= 1.0: area = abs((x2 - x1) * (y2 - y1))
                    else: area = abs((x2 - x1) * (y2 - y1)) / (img_w * img_h)
                else: area = 0.0
                detecciones.append((class_id, conf, area))
        elif arr.ndim == 2 and arr.shape[-1] == 85:
            for row in arr:
                obj_conf = float(row[4])
                if obj_conf < DETECT_CONF_MIN * 0.5: continue
                class_id = int(np.argmax(row[5:]))
                conf = obj_conf * float(row[5 + class_id])
                if conf < DETECT_CONF_MIN: continue
                x_c, y_c, bw, bh = row[0], row[1], row[2], row[3]
                area = float(bw * bh)
                if x_c > 1.0: area /= (img_w * img_h)
                detecciones.append((class_id, conf, area))
    return detecciones

def _es_stop(clase_id):
    if DETECT_CLASE_STOP is None: return False
    return int(clase_id) == int(DETECT_CLASE_STOP)

def _loop_deteccion_inner(uart):
    try:
        from hailo_platform import HEF, VDevice, InputVStreamParams, OutputVStreamParams, InferVStreams
    except ImportError: return

    script_dir = os.path.dirname(os.path.abspath(__file__))
    detect_path = os.path.join(script_dir, MODELO_DETECT)
    if not os.path.exists(detect_path): return

    try:
        target  = VDevice()
        hef     = HEF(detect_path)
        ng      = target.configure(hef)[0]
        in_name = hef.get_input_vstream_infos()[0].name
    except Exception: return

    primer_frame = True
    stop_cooldown_end = 0.0   

    with ng.activate():
        params_in  = InputVStreamParams.make(ng)
        params_out = OutputVStreamParams.make(ng)
        with InferVStreams(ng, params_in, params_out) as pipe:
            while not cerebro.stop_event.is_set():
                got_frame = cerebro.frame_detect_event.wait(timeout=0.5)
                cerebro.frame_detect_event.clear()
                if not got_frame: continue
                with cerebro.frame_detect_lock: frame_rgb = cerebro.frame_detect
                if frame_rgb is None: continue

                img_h, img_w = frame_rgb.shape[:2]
                try:
                    det_in_info = hef.get_input_vstream_infos()[0]
                    det_h = det_in_info.shape[1] if len(det_in_info.shape) >= 2 else 640
                    det_w = det_in_info.shape[2] if len(det_in_info.shape) >= 3 else 640
                except Exception: det_h, det_w = 640, 640

                img_resized = cv2.resize(frame_rgb, (det_w, det_h))
                input_data  = {in_name: np.expand_dims(img_resized, axis=0).astype(np.uint8)}
                try: output = pipe.infer(input_data)
                except Exception: continue

                dets = _parse_detecciones(output, img_h, img_w, primer_frame)
                primer_frame = False
                ahora = time.time()

                if ahora < stop_cooldown_end: continue

                mejor_area = 0.0
                for (clase_id, conf, area) in dets:
                    if _es_stop(clase_id) and area > mejor_area: mejor_area = area

                estado_actual = cerebro.stop_state

                if estado_actual == "normal":
                    if mejor_area >= DETECT_AREA_APROX:
                        cerebro.stop_state = "aproximando"
                        cerebro.stop_state_since = ahora
                        enviar_comando("HAZON", uart)
                        enviar_comando("SET:driveSpeed={}".format(VELOCIDAD_APROX), uart)

                elif estado_actual == "aproximando":
                    if mejor_area >= DETECT_AREA_STOP:
                        cerebro.parada_activa = True
                        cerebro.stop_state = "detenido"
                        cerebro.stop_state_since = ahora
                        enviar_comando("STOP", uart) 
                        print("[STOP] Señal detectada. Frenando y encendiendo luces 41. Esperando {}s".format(STOP_ESPERA_S))
                        
                    elif mejor_area < DETECT_AREA_APROX * 0.5:
                        cerebro.stop_state = "normal"
                        enviar_comando("HAZOFF", uart)
                        enviar_comando("SET:driveSpeed={}".format(VELOCIDAD_AUTONOMA), uart)

                elif estado_actual == "detenido":
                    if (ahora - cerebro.stop_state_since) >= STOP_ESPERA_S:
                        cerebro.stop_state = "cooldown"
                        cerebro.parada_activa = False
                        enviar_comando("HAZOFF", uart)
                        enviar_comando("SET:driveSpeed={}".format(VELOCIDAD_AUTONOMA), uart)
                        enviar_comando("W", uart)
                        cerebro.ultimo_comando = "W"
                        stop_cooldown_end = ahora + STOP_COOLDOWN_S

                elif estado_actual == "cooldown":
                    if ahora >= stop_cooldown_end: cerebro.stop_state = "normal"

def loop_deteccion(uart):
    try: _loop_deteccion_inner(uart)
    except Exception: pass

def _loop_inteligencia_inner(uart):
    try: from picamera2 import Picamera2
    except ImportError: raise RuntimeError("Falta picamera2")
    try: from hailo_platform import HEF, VDevice, InputVStreamParams, OutputVStreamParams, InferVStreams
    except ImportError: raise RuntimeError("Falta hailo")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    modelo_path = os.path.join(script_dir, MODELO_LANE)

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration({"size": (640, 480)}))
    picam2.start()

    target = VDevice()
    hef = HEF(modelo_path)
    network_group = target.configure(hef)[0]
    input_name = hef.get_input_vstream_infos()[0].name

    enviar_comando("SET:driveSpeed={}".format(VELOCIDAD_AUTONOMA), uart)

    blackout_cnt = 0; recover_cnt = 0; frames_total = 0
    lf_start_time = 0; LF_TIMEOUT_S = 10       
    cmd_candidato_prev = ""; confirmaciones = 0      

    with network_group.activate():
        params = [InputVStreamParams.make(network_group), OutputVStreamParams.make(network_group)]
        with InferVStreams(network_group, *params) as pipe:
            while not cerebro.stop_event.is_set():
                frame_raw = picam2.capture_array()
                frames_total += 1
                if frame_raw.shape[-1] == 4: img_rgb = cv2.cvtColor(frame_raw, cv2.COLOR_RGBA2RGB)
                else: img_rgb = frame_raw

                img_resized = cv2.resize(img_rgb, (IMG_SIZE, IMG_SIZE))
                input_data = {input_name: np.expand_dims(img_resized, axis=0).astype(np.uint8)}
                out = pipe.infer(input_data)
                mask = list(out.values())[0][0].astype(np.float32)

                h, w = mask.shape[:2]
                row_h = h // GRID_FILAS; col_w = w // GRID_COLS
                col_pos = np.linspace(-1.0, 1.0, GRID_COLS)

                numerador = 0.0; denominador = 0.0; actividad_total = 0.0

                for r in range(GRID_FILAS):
                    peso_r = PESOS_FILAS[r]
                    r0 = r * row_h; r1 = (r + 1) * row_h if r < GRID_FILAS - 1 else h
                    for c in range(GRID_COLS):
                        c0 = c * col_w; c1 = (c + 1) * col_w if c < GRID_COLS - 1 else w
                        suma_celda = float(np.sum(mask[r0:r1, c0:c1]))
                        actividad_total += suma_celda
                        celda_ponderada  = suma_celda * peso_r
                        numerador   += celda_ponderada * col_pos[c]
                        denominador += celda_ponderada

                actividad_norm = actividad_total / (h * w) if h * w > 0 else 0.0
                if denominador > 0.0: centroid_error = float(np.clip(numerador / denominador, -1.0, 1.0))
                else: centroid_error = 0.0

                if actividad_norm < UMBRAL_ACTIVIDAD:
                    blackout_cnt += 1
                    recover_cnt = 0
                    if blackout_cnt >= BLACKOUT_FRAMES and not cerebro.modo_LF:
                        enviar_comando("LF", uart)
                        cerebro.modo_LF = True
                        cerebro.ultimo_comando = "LF"
                        lf_start_time = time.time()
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
                                lf_start_time = 0
                        else: recover_cnt = 0

                if cerebro.modo_LF and lf_start_time > 0:
                    if (time.time() - lf_start_time) > LF_TIMEOUT_S:
                        enviar_comando("NOLF", uart)
                        cerebro.modo_LF = False
                        cerebro.ultimo_comando = ""
                        lf_start_time = 0
                        blackout_cnt = 0

                if frames_total % DETECT_FEED_INTERVALO == 0:
                    with cerebro.frame_detect_lock: cerebro.frame_detect = img_rgb.copy()
                    cerebro.frame_detect_event.set()

                if not cerebro.modo_LF and not cerebro.parada_activa:
                    if centroid_error < -UMBRAL_ERROR_GIRO: nuevo_cmd = "A"   
                    elif centroid_error > UMBRAL_ERROR_GIRO: nuevo_cmd = "D"   
                    else: nuevo_cmd = "W"   

                    if nuevo_cmd == cmd_candidato_prev: confirmaciones += 1
                    else: confirmaciones = 1; cmd_candidato_prev = nuevo_cmd

                    # Latido constante: La Pi "bombardea" el ESP32 para mantenerlo vivo
                    if confirmaciones >= CONFIRMACIONES_CAMBIO:
                        enviar_comando(nuevo_cmd, uart)
                        cerebro.ultimo_comando = nuevo_cmd
                        confirmaciones = CONFIRMACIONES_CAMBIO 

                frame_viz = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                for r in range(1, GRID_FILAS):
                    y = r * (480 // GRID_FILAS); cv2.line(frame_viz, (0, y), (640, y), (80, 80, 80), 1)
                for c in range(1, GRID_COLS):
                    x = c * (640 // GRID_COLS); cv2.line(frame_viz, (x, 0), (x, 480), (255, 255, 0), 1)

                cx = int((centroid_error + 1.0) / 2.0 * 640)
                cv2.line(frame_viz, (cx, 0), (cx, 480), (0, 255, 0), 2)
                cv2.line(frame_viz, (320, 0), (320, 480), (0, 0, 200), 1)

                color_orden = (0, 255, 0) if not cerebro.modo_LF else (0, 165, 255)
                cv2.putText(frame_viz, "ORDEN: " + cerebro.ultimo_comando, (20, 60), cv2.FONT_HERSHEY_PLAIN, 3, color_orden, 4)
                
                _, buffer = cv2.imencode(".jpg", frame_viz, [cv2.IMWRITE_JPEG_QUALITY, 50])
                with cerebro.lock: cerebro.frame_web = buffer.tobytes()

    picam2.stop()

def test_uart_y_camara(): pass
def test_drive(): pass

app = FastAPI()

@app.get("/")
def video_feed():
    def generate():
        while True:
            with cerebro.lock:
                if cerebro.frame_web: yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + cerebro.frame_web + b"\r\n")
            time.sleep(0.04)
    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/telemetria")
def telemetria():
    return { "esp": cerebro.telemetria_esp, "modo_LF": cerebro.modo_LF, "error": cerebro.error_fatal }

@app.get("/status")
def status():
    return { "vivo": True, "ultimo_comando": cerebro.ultimo_comando, "modo_LF": cerebro.modo_LF, "error": cerebro.error_fatal }

def shutdown(signum, frame):
    cerebro.stop_event.set()
    if _uart_global is not None:
        enviar_comando("X", _uart_global)
        enviar_comando("NOLF", _uart_global)
    time.sleep(0.2)
    sys.exit(0)

_uart_global = None

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--test": sys.exit(0)
    if len(sys.argv) > 1 and sys.argv[1] == "--test-drive": sys.exit(0)

    _uart_global = abrir_uart()
    signal.signal(signal.SIGINT, shutdown); signal.signal(signal.SIGTERM, shutdown)

    threading.Thread(target=loop_inteligencia, args=(_uart_global,), daemon=True).start()
    threading.Thread(target=loop_lectura_telemetria, args=(_uart_global,), daemon=True).start()
    threading.Thread(target=loop_deteccion, args=(_uart_global,), daemon=True).start()

    time.sleep(10)
    if cerebro.error_fatal: sys.exit(1)
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="error")