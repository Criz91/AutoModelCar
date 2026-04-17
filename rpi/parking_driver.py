#!/usr/bin/env python3
"""
AutoModelCar TMR 2026 - Controlador Raspberry Pi
=================================================

Este script corre en la Raspberry Pi y controla el vehiculo durante la
competencia. Se comunica con la ESP32-S3 por UART2 a 115200 bps.

CONEXION FISICA:
    Pi GPIO 14 (TX) --> ESP32 pin 16 (RX2)
    Pi GPIO 15 (RX) --> ESP32 pin 17 (TX2)
    GND comun entre Pi y ESP32

PUERTO SERIE EN RASPBERRY PI:
    - Deshabilitar consola serial: sudo raspi-config -> Interfacing Options -> Serial
      (deshabilitar "login shell", HABILITAR "serial port hardware")
    - Puerto: /dev/serial0 (o /dev/ttyAMA0)

COMO USAR:
    python3 parking_driver.py              # modo normal
    python3 parking_driver.py --debug      # muestra toda la telemetria recibida
    python3 parking_driver.py --manual     # modo manual: solo reenviar teclas al ESP

ESTADOS DEL CONTROLADOR (estado_pi):
    PISTA        - Siguiendo el circuito; la IA manda W/A/D/S
    STOP_SENIAL  - Se envio STOP al ESP; esperando que vuelva a MANUAL (6s)
    PARKING      - Se envio PR al ESP; ESP maneja el estacionamiento autonomamente
    FIN          - Maniobra completada o abortada

PROTOCOLO DE COMANDOS (lo que la Pi manda al ESP):
    W            - Traccion hacia adelante
    S            - Traccion reversa
    A            - Gira direccion izquierda
    D            - Gira direccion derecha
    TX           - Para solo traccion (no toca direccion)
    C            - Para solo direccion (no toca traccion)
    X            - Para ambos motores
    STOP         - Señal de transito: para 6s con intermitentes, luego MANUAL
    ESTOP        - Emergencia: para todo permanentemente
    PR           - Inicia maniobra de estacionamiento autonoma (Fase B)
    TC           - Inicia test ciego (Fase A, solo debug)
    SET:clave=valor - Ajusta parametro en tiempo real (ej. SET:velocidadAvance=220)

TELEMETRIA JSON QUE ENVIA EL ESP (cada 200ms):
    {
      "modo":    "MANUAL" | "TEST_CIEGO" | "ESTACIONAR_DER" | "ESTACIONADO" |
                 "ABORTADO" | "PARADA_SENIAL",
      "estado":  "NINGUNO" | "TC_*" | "PB_*",
      "tEstado": ms en el estado actual,
      "dR":      distancia derecha (cm), 999 = sin lectura,
      "dL":      distancia izquierda (cm),
      "dB":      distancia trasera (cm),
      "dF":      distancia frontal (cm),
      "velTrac": PWM traccion actual (0-255),
      "posDir":  posicion estimada de direccion (ms),
      "inter":   "OFF" | "IZQ" | "DER" | "AMBAS" | "FIJAS",
      "freno":   true/false,
      "hueco":   largo del hueco medido (cm, solo durante Fase B)
    }

DONDE CONECTAR LA IA:
    Las tres funciones marcadas con "TODO" son los puntos de integracion:
      - decision_ia()        -> salida del modelo de segmentacion de carril
      - detectar_stop()      -> deteccion de señal de STOP por vision
      - es_zona_parking()    -> decision de cuando iniciar el estacionamiento
"""

import serial
import json
import threading
import time
import queue
import sys
import argparse

# ── Configuracion de puerto UART ────────────────────────────────────────────
UART_PORT = "/dev/serial0"   # Cambiar a /dev/ttyAMA0 si serial0 no existe
UART_BAUD = 115200

# ── Resolucion de camara para el modelo de IA ────────────────────────────────
# El modelo de segmentacion de carril (Hailo) recibe frames de 640x640 px.
# Usa esta constante al capturar o redimensionar frames con OpenCV:
#   frame_resized = cv2.resize(frame, (CAM_W, CAM_H))
CAM_W = 640
CAM_H = 640

# ── Intervalos de control ───────────────────────────────────────────────────
CMD_INTERVAL_S  = 0.10   # Enviar comando de movimiento cada 100ms
LOOP_SLEEP_S    = 0.02   # Velocidad del loop principal (50 Hz)
STATUS_INTERVAL = 0.50   # Imprimir estado en pantalla cada 500ms

# ── Estados del controlador Pi ──────────────────────────────────────────────
PISTA       = "PISTA"
STOP_SENIAL = "STOP_SENIAL"
PARKING     = "PARKING"
FIN         = "FIN"


# ============================================================================
#  SECCION DE IA — MODIFICAR AQUI
#  El equipo de vision conecta sus modelos en estas tres funciones.
# ============================================================================

def decision_ia(tel: dict) -> str | None:
    """
    Retorna el comando de movimiento para el siguiente tick.

    Valores validos: "W", "A", "S", "D", "X" o None (no enviar nada en este tick).

    Protocolo de comandos al ESP32:
        "W" = traccion adelante (la direccion queda como estaba)
        "S" = traccion reversa  (la direccion queda como estaba)
        "A" = girar izquierda; si el carro estaba parado, tambien arranca traccion
        "D" = girar derecha;   si el carro estaba parado, tambien arranca traccion
        "X" = para todo (traccion + direccion)

    Parametro tel: diccionario con la telemetria actual del ESP.
    Campos utiles:
        tel["dR"]  - distancia sensor derecho (cm)
        tel["dL"]  - distancia sensor izquierdo (cm)
        tel["dF"]  - distancia sensor frontal (cm)
        tel["dB"]  - distancia sensor trasero (cm)

    COMO CONECTAR EL MODELO HAILO (camara 640x640):
    -----------------------------------------------
    1. Importar el modelo al inicio del archivo:
           from hailo_inference import HailoModel
           modelo = HailoModel("segmentacion_carril.hef")

    2. Capturar y redimensionar frame:
           ret, frame = camara.read()
           frame_in = cv2.resize(frame, (CAM_W, CAM_H))  # 640x640

    3. Correr inferencia y decidir:
           resultado = modelo.inferir(frame_in)
           if resultado.desviacion_izquierda > 0.3:  return "A"
           if resultado.desviacion_derecha  > 0.3:  return "D"
           return "W"
    """
    # STUB: siempre adelante — reemplazar con logica real
    return "W"


def detectar_stop(tel: dict) -> bool:
    """
    Retorna True cuando se detecta una señal de STOP y el vehiculo
    esta a la distancia correcta para detenerse (~5 cm del letrero).

    La Pi envia STOP al ESP, que detiene el vehiculo por 6 segundos
    con intermitentes encendidas y luego vuelve a MANUAL automaticamente.

    TODO: conectar aqui el detector de señales de transito.
    Ejemplo:
        bbox, clase = detector_señales.detectar(frame_camara)
        if clase == "STOP" and distancia_cm(bbox) <= 5:
            return True
        return False
    """
    # STUB: nunca detecta stop — reemplazar con logica real
    return False


def es_zona_parking(tel: dict) -> bool:
    """
    Retorna True cuando la Pi decide que es el momento de iniciar
    la maniobra de estacionamiento (enviar PR al ESP).

    Cuando esto retorna True, la Pi deja de mandar WASD y el ESP
    toma control autonomo usando sus sensores ultrasonicos.

    Criterios posibles (elegir el que aplique):
      - Distancia recorrida ~1 metro desde el inicio del circuito
      - Vision detecta la zona de parking
      - El sensor derecho ya vio el primer carro vecino

    TODO: implementar logica de deteccion de zona de parking.
    Ejemplo basado en sensor derecho del ESP:
        return tel.get("dR", 999) <= 15  # carro detectado a <15cm
    """
    # STUB: nunca inicia parking — reemplazar con logica real
    return False


# ============================================================================
#  CLASE PRINCIPAL
# ============================================================================

class ControladorPi:
    def __init__(self, debug: bool = False):
        self.debug     = debug
        self.ser       = None
        self._cola     = queue.Queue(maxsize=5)
        self.tel       = {}          # Ultima telemetria recibida
        self.activo    = True
        self._hilo_rx  = None

    def conectar(self) -> bool:
        """Abre el puerto UART. Retorna True si tuvo exito."""
        try:
            self.ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0.1)
            print(f"[PI] UART abierto: {UART_PORT} @ {UART_BAUD} bps")
            self._hilo_rx = threading.Thread(target=self._leer_uart, daemon=True)
            self._hilo_rx.start()
            return True
        except serial.SerialException as e:
            print(f"[PI] ERROR abriendo UART: {e}")
            print(f"[PI] Verifica que {UART_PORT} existe y tienes permisos (sudo o grupo dialout)")
            return False

    def enviar(self, cmd: str):
        """Envia un comando al ESP terminado en newline."""
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\n").encode())
            print(f"[PI > ESP] {cmd}")

    def _leer_uart(self):
        """Hilo: lee bytes del UART, parsea JSON, pone en cola."""
        buf = ""
        while self.activo:
            try:
                if self.ser.in_waiting:
                    c = self.ser.read().decode(errors="ignore")
                    if c == "\n":
                        linea = buf.strip()
                        buf = ""
                        if linea.startswith("{"):
                            try:
                                data = json.loads(linea)
                                # Mantener solo la lectura mas reciente
                                if self._cola.full():
                                    try:
                                        self._cola.get_nowait()
                                    except queue.Empty:
                                        pass
                                self._cola.put_nowait(data)
                            except json.JSONDecodeError:
                                pass
                        elif linea:
                            # Mensajes de debug del ESP (no JSON)
                            print(f"[ESP] {linea}")
                    else:
                        buf += c
                        if len(buf) > 400:
                            buf = ""   # Descarta lineas corruptas
                else:
                    time.sleep(0.005)
            except Exception as e:
                if self.activo:
                    print(f"[PI] Error leyendo UART: {e}")
                time.sleep(0.02)

    def obtener_tel(self) -> dict:
        """Retorna la telemetria mas reciente. No bloqueante."""
        try:
            while not self._cola.empty():
                self.tel = self._cola.get_nowait()
        except queue.Empty:
            pass
        if self.debug and self.tel:
            print(f"[TEL] {self.tel}")
        return self.tel

    def detener(self):
        """Para todo y cierra el puerto."""
        self.activo = False
        try:
            self.enviar("ESTOP")
        except Exception:
            pass
        if self.ser:
            time.sleep(0.1)
            try:
                self.ser.close()
            except Exception:
                pass
        print("[PI] UART cerrado.")


# ============================================================================
#  MODO MANUAL (para testing desde teclado)
# ============================================================================

def modo_manual(ctrl: ControladorPi):
    """
    Permite controlar el vehiculo con el teclado directamente.
    Util para verificar la comunicacion antes de integrar la IA.

    Teclas: W/A/S/D = movimiento, X = stop total, Q = salir
    """
    print("\n[MANUAL] Teclas: W/A/S/D = mover, X = parar, Q = salir")
    print("[MANUAL] Presiona Enter despues de cada tecla\n")
    import tty, termios
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            tel = ctrl.obtener_tel()
            if tel:
                modo = tel.get("modo", "?")
                dR   = tel.get("dR", 999)
                dB   = tel.get("dB", 999)
                print(f"\r[TEL] Modo:{modo:15s} dR:{dR:5.0f}cm  dB:{dB:5.0f}cm   ", end="")
            c = sys.stdin.read(1).upper()
            if c in ("W", "A", "S", "D", "X"):
                ctrl.enviar(c)
            elif c == "Q":
                break
            time.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


# ============================================================================
#  LOOP PRINCIPAL DE COMPETENCIA
# ============================================================================

def run(debug: bool = False):
    ctrl = ControladorPi(debug=debug)

    if not ctrl.conectar():
        sys.exit(1)

    print("[PI] Esperando 2 segundos antes de iniciar control...")
    time.sleep(2)

    estado_pi      = PISTA
    t_ultimo_cmd   = 0.0
    t_ultimo_status = 0.0
    ultimo_cmd_env  = "---"

    print(f"[PI] Iniciando. Estado: {estado_pi}")
    print("[PI] Ctrl+C para parada de emergencia\n")
    print(f"{'ESTADO-PI':12s} {'MODO-ESP':16s} {'EST-ESP':20s} {'dR':>6} {'dL':>6} {'dB':>6} {'CMD':>5}")
    print("-" * 80)

    try:
        while True:
            tel        = ctrl.obtener_tel()
            ahora      = time.time()
            modo_esp   = tel.get("modo",   "")
            estado_esp = tel.get("estado", "")

            # Imprimir estado en pantalla periodicamente
            if ahora - t_ultimo_status >= STATUS_INTERVAL and tel:
                dR = tel.get("dR", 999)
                dL = tel.get("dL", 999)
                dB = tel.get("dB", 999)
                dR_str = f"{dR:5.0f}" if dR < 999 else "  ---"
                dL_str = f"{dL:5.0f}" if dL < 999 else "  ---"
                dB_str = f"{dB:5.0f}" if dB < 999 else "  ---"
                print(f"{estado_pi:12s} {modo_esp:16s} {estado_esp:20s} {dR_str} {dL_str} {dB_str} {ultimo_cmd_env:>5}")
                t_ultimo_status = ahora

            # ── Fin de carrera ─────────────────────────────────────────────
            if modo_esp == "ESTACIONADO" or estado_esp == "PB_ESTACIONADO":
                print("\n[PI] ESTACIONADO correctamente. Fin de la maniobra.")
                estado_pi = FIN
                break

            if modo_esp == "ABORTADO" or estado_esp == "PB_ABORTADO":
                hueco = tel.get("hueco", 0)
                print(f"\n[PI] ESP aborto la maniobra. Hueco medido: {hueco:.1f} cm")
                estado_pi = FIN
                break

            # ── Mientras ESP maneja autonomamente, no interferir ───────────
            if modo_esp in ("ESTACIONAR_DER", "PARADA_SENIAL"):
                time.sleep(LOOP_SLEEP_S)
                continue

            # ── Cuando ESP vuelve a MANUAL despues del STOP ────────────────
            if estado_pi == STOP_SENIAL:
                if modo_esp == "MANUAL":
                    print("[PI] PARADA_SENIAL completada. Reanudando pista.")
                    estado_pi = PISTA
                time.sleep(LOOP_SLEEP_S)
                continue

            # ── Esperando que ESP termine el parking ───────────────────────
            if estado_pi == PARKING:
                time.sleep(LOOP_SLEEP_S)
                continue

            # ── Siguiendo la pista ─────────────────────────────────────────
            if estado_pi == PISTA:

                # Deteccion de señal STOP (delega a funcion de IA)
                if detectar_stop(tel):
                    print("[PI] Señal STOP detectada. Enviando STOP.")
                    ctrl.enviar("STOP")
                    ultimo_cmd_env = "STOP"
                    estado_pi = STOP_SENIAL
                    time.sleep(LOOP_SLEEP_S)
                    continue

                # Decision de iniciar el estacionamiento (delega a funcion de IA)
                if es_zona_parking(tel):
                    print("[PI] Zona de parking detectada. Enviando PR.")
                    ctrl.enviar("PR")
                    ultimo_cmd_env = "PR"
                    estado_pi = PARKING
                    time.sleep(LOOP_SLEEP_S)
                    continue

                # Enviar comando de movimiento periodicamente
                if ahora - t_ultimo_cmd >= CMD_INTERVAL_S:
                    cmd = decision_ia(tel)
                    if cmd:
                        ctrl.enviar(cmd)
                        ultimo_cmd_env = cmd
                    t_ultimo_cmd = ahora

            time.sleep(LOOP_SLEEP_S)

    except KeyboardInterrupt:
        print("\n[PI] Ctrl+C: enviando ESTOP.")
    finally:
        ctrl.detener()


# ============================================================================
#  PUNTO DE ENTRADA
# ============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="AutoModelCar TMR 2026 - Controlador Raspberry Pi"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Imprime toda la telemetria JSON recibida del ESP"
    )
    parser.add_argument(
        "--manual",
        action="store_true",
        help="Modo manual: controla con teclado W/A/S/D (para probar la comunicacion)"
    )
    parser.add_argument(
        "--port",
        default=UART_PORT,
        help=f"Puerto serie (default: {UART_PORT})"
    )
    args = parser.parse_args()

    # Permitir sobreescribir el puerto desde linea de comandos
    UART_PORT = args.port

    ctrl = ControladorPi(debug=args.debug)
    if not ctrl.conectar():
        sys.exit(1)

    time.sleep(1)

    if args.manual:
        try:
            modo_manual(ctrl)
        finally:
            ctrl.detener()
    else:
        run(debug=args.debug)
