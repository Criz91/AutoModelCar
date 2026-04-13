import asyncio
import json
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import websockets

# Configuracion de red del ESP32
ESP32_IP = "192.168.4.1"
WS_URL = f"ws://{ESP32_IP}/ws"

# Definicion de los parametros ajustables en la interfaz
# Formato: (Identificador, Etiqueta en UI, Valor Minimo, Valor Maximo, Valor por Defecto)
SLIDERS = [
    ("driveSpeed",       "Velocidad manual (PWM)",        0,    255,  180),
    ("parkDriveSpeed",   "Velocidad estacionar (PWM)",    0,    255,  130),
    ("steerSpeed",       "Velocidad direccion (PWM)",     0,    255,  230),
    ("tSteerFullMs",     "Tiempo direccion tope-tope ms", 100,  1500, 400),
    ("steerTrimMs",      "Trim centro direccion ms",     -150,  150,  0),
    ("distCarroCm",      "Dist. ve carro (<) cm",         5,    100,  35),
    ("distHuecoCm",      "Dist. ve hueco (>) cm",         10,   200,  45),
    ("distFrenaSuaveCm", "Dist. frena suave (banqueta)",  3,    50,   15),
    ("distParaYaCm",     "Dist. para ya (banqueta)",      2,    30,   8),
    ("tAvanzarHuecoMs",  "Avance tras hueco (ms)",        0,    5000, 700),
    ("minHuecoStableMs", "Hueco estable min (ms)",        0,    2000, 250),
]

# Diccionario de colores para el indicador visual de estado del vehiculo
STATE_COLORS = {
    "MANUAL":  "#9e9e9e",  # Gris
    "TEST":    "#42a5f5",  # Azul
    "AUTO":    "#ffb300",  # Ambar (maniobrando)
    "DONE":    "#43a047",  # Verde
    "ABORT":   "#e53935",  # Rojo
    "?":       "#9e9e9e",  # Gris por defecto
}

class CarControllerGUI:
    def __init__(self, root):
        # Configuracion de la ventana principal
        self.root = root
        self.root.title("AutoModelCar - Panel de Control")
        self.root.geometry("1180x680")
        self.root.configure(bg="#1e1e1e")

        # Configuracion del tema visual oscuro para los controles (ttk)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background="#1e1e1e", foreground="#e0e0e0", fieldbackground="#2a2a2a")
        style.configure("TLabel", background="#1e1e1e", foreground="#e0e0e0")
        style.configure("TLabelframe", background="#1e1e1e", foreground="#e0e0e0")
        style.configure("TLabelframe.Label", background="#1e1e1e", foreground="#90caf9", font=("Segoe UI", 10, "bold"))
        style.configure("TFrame", background="#1e1e1e")
        style.configure("TButton", padding=6, font=("Segoe UI", 9))
        style.configure("Big.TButton", padding=10, font=("Segoe UI", 11, "bold"))
        
        # Estilo especifico para el boton de paro de emergencia
        style.configure("Estop.TButton", padding=14, font=("Segoe UI", 13, "bold"), background="#e53935", foreground="white")
        style.map("Estop.TButton", background=[("active", "#ff5252")])

        # Variables de estado de la conexion
        self.ws = None
        self.connected = False
        self.pressed = set() # Registro de teclas presionadas para evitar repeticion

        # Inicializacion de variables para los sliders basados en la configuracion
        self.slider_vars = {}
        for k, _, _, _, default in SLIDERS:
            self.slider_vars[k] = tk.IntVar(value=default)

        # Construccion de la interfaz de usuario
        self._build_ui()

        # Configuracion del hilo secundario para manejar websockets sin congelar la interfaz
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        # Vinculacion de eventos de teclado (presionar y soltar)
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

        # Evento para cerrar el programa limpiamente
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        # Contenedor principal
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        # Division en tres columnas proporcionales
        outer.columnconfigure(0, weight=1, uniform="col")
        outer.columnconfigure(1, weight=1, uniform="col")
        outer.columnconfigure(2, weight=1, uniform="col")
        outer.rowconfigure(0, weight=1)

        # Asignacion de frames a las columnas
        col_left   = ttk.Frame(outer)
        col_left.grid(row=0, column=0, sticky="nsew", padx=4)
        
        col_mid    = ttk.Frame(outer)
        col_mid.grid(row=0, column=1, sticky="nsew", padx=4)
        
        col_right  = ttk.Frame(outer)
        col_right.grid(row=0, column=2, sticky="nsew", padx=4)

        # Llamadas para construir cada seccion
        self._build_left(col_left)
        self._build_mid(col_mid)
        self._build_right(col_right)

    def _build_left(self, parent):
        # Panel de conexion
        conn = ttk.LabelFrame(parent, text="Conexion", padding=10)
        conn.pack(fill="x", pady=(0, 8))

        self.lbl_status = ttk.Label(conn, text="Desconectado", font=("Segoe UI", 10, "bold"), foreground="#e57373")
        self.lbl_status.pack(side="left")

        ttk.Button(conn, text="Conectar",    command=self.connect   ).pack(side="right", padx=3)
        ttk.Button(conn, text="Desconectar", command=self.disconnect).pack(side="right", padx=3)

        # Panel de estado general (muestra MANUAL, AUTO, ABORT, etc.)
        st = ttk.LabelFrame(parent, text="Estado", padding=10)
        st.pack(fill="x", pady=(0, 8))

        self.state_canvas = tk.Canvas(st, height=70, bg="#9e9e9e", highlightthickness=0)
        self.state_canvas.pack(fill="x")
        self.state_text = self.state_canvas.create_text(10, 35, anchor="w", text="MANUAL", font=("Segoe UI", 18, "bold"), fill="white")
        self.substate_text = self.state_canvas.create_text(10, 58, anchor="w", text="-", font=("Segoe UI", 9), fill="white")

        # Panel de lectura de sensores
        sens = ttk.LabelFrame(parent, text="Sensores (cm)", padding=10)
        sens.pack(fill="x", pady=(0, 8))

        self.lbl_R = self._mk_big_label(sens, "Derecho",   "999")
        self.lbl_L = self._mk_big_label(sens, "Izquierdo", "999")
        self.lbl_B = self._mk_big_label(sens, "Trasero",   "999")

        # Panel de posicion estimada de direccion
        dr = ttk.LabelFrame(parent, text="Direccion (estimada)", padding=10)
        dr.pack(fill="x", pady=(0, 8))

        self.lbl_steer_pos = ttk.Label(dr, text="200 ms", font=("Segoe UI", 12, "bold"))
        self.lbl_steer_pos.pack(anchor="w")

        # Barra visual para indicar la posicion de la direccion
        self.steer_bar = tk.Canvas(dr, height=20, bg="#2a2a2a", highlightthickness=0)
        self.steer_bar.pack(fill="x", pady=(4, 0))
        self.steer_bar_rect = self.steer_bar.create_rectangle(0, 0, 0, 20, fill="#42a5f5", width=0)

    def _mk_big_label(self, parent, name, initial):
        # Funcion auxiliar para crear etiquetas grandes de sensores
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=name + ":", width=10).pack(side="left")
        lbl = ttk.Label(row, text=initial, font=("Consolas", 16, "bold"), foreground="#90caf9")
        lbl.pack(side="left")
        return lbl

    def _build_mid(self, parent):
        # Panel de controles manuales con botones
        ctrl = ttk.LabelFrame(parent, text="Control manual (WASD)", padding=10)
        ctrl.pack(fill="x", pady=(0, 8))

        # Botones de traccion (W, X, S)
        row1 = ttk.Frame(ctrl)
        row1.pack(fill="x", pady=3)
        ttk.Button(row1, text="W\nForward", style="Big.TButton", command=lambda: self.set_drive("W")).pack(side="left", expand=True, fill="both", padx=2)
        ttk.Button(row1, text="X\nStop",    style="Big.TButton", command=lambda: self.set_drive("X")).pack(side="left", expand=True, fill="both", padx=2)
        ttk.Button(row1, text="S\nReverse", style="Big.TButton", command=lambda: self.set_drive("S")).pack(side="left", expand=True, fill="both", padx=2)

        # Botones de direccion (A, C, D)
        row2 = ttk.Frame(ctrl)
        row2.pack(fill="x", pady=3)
        ttk.Button(row2, text="A\nLeft",   style="Big.TButton", command=lambda: self.set_steer("A")).pack(side="left", expand=True, fill="both", padx=2)
        ttk.Button(row2, text="C\nCenter", style="Big.TButton", command=lambda: self.set_steer("C")).pack(side="left", expand=True, fill="both", padx=2)
        ttk.Button(row2, text="D\nRight",  style="Big.TButton", command=lambda: self.set_steer("D")).pack(side="left", expand=True, fill="both", padx=2)

        # Panel de modo autonomo y herramientas
        auto = ttk.LabelFrame(parent, text="Modo autonomo", padding=10)
        auto.pack(fill="x", pady=(0, 8))

        prow = ttk.Frame(auto)
        prow.pack(fill="x", pady=3)
        ttk.Button(prow, text="Estacionar IZQUIERDA", style="Big.TButton", command=lambda: self.send("PL")).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(prow, text="Estacionar DERECHA",   style="Big.TButton", command=lambda: self.send("PR")).pack(side="left", expand=True, fill="x", padx=2)

        urow = ttk.Frame(auto)
        urow.pack(fill="x", pady=3)
        ttk.Button(urow, text="Sensor Test (T)", command=lambda: self.send("T")  ).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(urow, text="Hazards (H)",     command=lambda: self.send("H")  ).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(urow, text="Recalibrar (CAL)",command=lambda: self.send("CAL")).pack(side="left", expand=True, fill="x", padx=2)

        # Boton principal de emergencia
        estop = ttk.Frame(parent)
        estop.pack(fill="x", pady=(8, 0))
        ttk.Button(estop, text="PARO DE EMERGENCIA", style="Estop.TButton", command=self.estop).pack(fill="x", ipady=6)

        # Texto informativo
        info = ttk.Label(parent, text="Tip: WASD con la ventana enfocada.\nCualquier tecla durante AUTO cancela.\nEsc = paro de emergencia.", justify="left", foreground="#9e9e9e")
        info.pack(anchor="w", pady=(8, 0))

    def _build_right(self, parent):
        # Panel derecho para configuracion dinamica (sliders)
        cal = ttk.LabelFrame(parent, text="Calibracion en vivo", padding=10)
        cal.pack(fill="both", expand=True)

        # Implementacion de un canvas con scrollbar para manejar muchos sliders
        canvas = tk.Canvas(cal, bg="#1e1e1e", highlightthickness=0)
        sb = ttk.Scrollbar(cal, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)

        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)

        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        # Generar controles dinamicos a partir de la lista
        for key, label, mn, mx, default in SLIDERS:
            self._mk_slider(inner, key, label, mn, mx, default)

    def _mk_slider(self, parent, key, label, mn, mx, default):
        # Funcion auxiliar para construir cada slider con su etiqueta
        frm = ttk.Frame(parent)
        frm.pack(fill="x", pady=4)

        head = ttk.Frame(frm)
        head.pack(fill="x")
        ttk.Label(head, text=label, font=("Segoe UI", 9)).pack(side="left")
        
        val_lbl = ttk.Label(head, text=str(default), font=("Consolas", 10, "bold"), foreground="#90caf9", width=6, anchor="e")
        val_lbl.pack(side="right")

        var = self.slider_vars[key]

        # Funcion llamada al mover el slider
        def on_change(_e=None):
            v = int(float(var.get()))
            val_lbl.config(text=str(v))
            self.send(f"SET:{key}={v}") # Envio inmediato al ESP32

        sc = ttk.Scale(frm, from_=mn, to=mx, orient="horizontal", variable=var, command=on_change)
        sc.pack(fill="x")

    def _run_loop(self):
        # Bucle de eventos para las tareas asincronas
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def connect(self):
        # Lanza el proceso de conexion de manera segura desde tkinter al hilo asincrono
        asyncio.run_coroutine_threadsafe(self._connect_ws(), self.loop)

    async def _connect_ws(self):
        try:
            if self.ws:
                await self.ws.close()
            # Intento de conexion al servidor websocket del coche
            self.ws = await websockets.connect(WS_URL)
            self.connected = True
            self._set_status(True)
            # Inicia el proceso para recibir datos constantemente
            asyncio.ensure_future(self._listen_loop())
        except Exception as e:
            self.connected = False
            self.ws = None
            self._set_status(False)
            # Retorno seguro de mensajes de error a la interfaz principal
            self.root.after(0, lambda: messagebox.showerror("Error", f"No se pudo conectar:\n{e}"))

    async def _listen_loop(self):
        # Bucle infinito para escuchar la telemetria mientras este conectado
        try:
            while self.ws:
                msg = await self.ws.recv()
                self._handle_telemetry(msg)
        except Exception:
            self.ws = None
            self.connected = False
            self._set_status(False)

    def disconnect(self):
        # Lanza la desconexion de manera segura
        asyncio.run_coroutine_threadsafe(self._disconnect_ws(), self.loop)

    async def _disconnect_ws(self):
        try:
            if self.ws:
                await self.ws.close()
        finally:
            self.ws = None
            self.connected = False
            self._set_status(False)

    def send(self, msg: str):
        # Envio de comandos al ESP32
        asyncio.run_coroutine_threadsafe(self._send(msg), self.loop)

    async def _send(self, msg: str):
        if not self.ws:
            return
        try:
            await self.ws.send(msg)
        except Exception:
            self.ws = None
            self.connected = False
            self._set_status(False)

    def _set_status(self, ok: bool):
        # Actualizacion visual del indicador de conexion (seguro para hilos)
        def _do():
            if ok:
                self.lbl_status.config(text="Conectado", foreground="#81c784")
            else:
                self.lbl_status.config(text="Desconectado", foreground="#e57373")
        self.root.after(0, _do)

    def _handle_telemetry(self, msg):
        # Procesamiento del JSON enviado por el coche
        try:
            j = json.loads(msg)
        except Exception:
            return
            
        if j.get("t") != "tel":
            return

        # Actualizacion de la UI (debe hacerse con after para evitar bloqueos)
        def _do():
            self.lbl_R.config(text=str(j.get("R", "?")))
            self.lbl_L.config(text=str(j.get("L", "?")))
            self.lbl_B.config(text=str(j.get("B", "?")))

            sp = int(j.get("sp", 0))
            self.lbl_steer_pos.config(text=f"{sp} ms")

            # Calculo matematico para animar la barra de direccion
            tmax = self.slider_vars["tSteerFullMs"].get() or 400
            ratio = max(0.0, min(1.0, sp / float(tmax)))
            w = self.steer_bar.winfo_width()
            self.steer_bar.coords(self.steer_bar_rect, 0, 0, w * ratio, 20)

            # Actualizacion del recuadro principal de estado
            mode = j.get("mode", "?")
            sub  = j.get("st", "-")
            color = STATE_COLORS.get(mode, "#9e9e9e")
            self.state_canvas.config(bg=color)
            self.state_canvas.itemconfig(self.state_text, text=mode)
            self.state_canvas.itemconfig(self.substate_text, text=sub)

        self.root.after(0, _do)

    # Funciones de puente para enviar las letras de control
    def set_drive(self, cmd):
        self.send(cmd)

    def set_steer(self, cmd):
        self.send(cmd)

    def estop(self):
        self.send("ESTOP")

    def on_key_press(self, event):
        # Captura eventos de teclado, ignorando la repeticion automatica del sistema
        k = event.keysym.lower()
        if k in self.pressed:
            return
        self.pressed.add(k)

        if k == "w":      self.set_drive("W")
        elif k == "s":    self.set_drive("S")
        elif k == "a":    self.set_steer("A")
        elif k == "d":    self.set_steer("D")
        elif k == "escape": self.estop()

    def on_key_release(self, event):
        # Detiene los motores al soltar la tecla
        k = event.keysym.lower()
        self.pressed.discard(k)

        if k in ("w", "s"):
            self.set_drive("X")
        if k in ("a", "d"):
            self.set_steer("C")

    def on_close(self):
        # Limpieza antes de cerrar el programa
        try:
            self.estop()
        except Exception:
            pass
        self.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = CarControllerGUI(root)
    root.mainloop()