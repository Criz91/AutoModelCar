import asyncio
import json
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import websockets

ESP32_IP = "192.168.4.1"
WS_URL = f"ws://{ESP32_IP}/ws"


# Lista de sliders de calibracion que aparecen en el panel derecho.
# Cada entrada es (clave, etiqueta visible, minimo, maximo, valor inicial).
# La clave es la misma que entiende el firmware en applyParam(),
# asi al mover el slider se manda "SET:clave=valor" por WebSocket
# y el ESP32 ajusta el parametro en vivo sin reflashear.
SLIDERS = [
    ("driveSpeed",       "Velocidad manual (PWM)",        0,    255,  180),
    ("parkDriveSpeed",   "Velocidad estacionar (PWM)",    0,    255,  200),
    ("steerSpeed",       "Velocidad direccion (PWM)",     0,    255,  170),
    ("tSteerFullMs",     "Tiempo direccion tope-tope ms", 100,  1500, 400),
    ("steerTrimMs",      "Trim centro direccion ms",     -150,  150,  0),
    ("tAvanceInicialMs", "Override avance inicial (ms)",  0,    10000, 0),
    ("distCarroCm",      "Dist. ve carro (<) cm",         5,    100,  25),
    ("distHuecoCm",      "Dist. ve hueco (>) cm",         10,   200,  45),
    ("distFrenaSuaveCm", "Dist. frena suave (banqueta)",  3,    50,   15),
    ("distParaYaCm",     "Dist. para ya (banqueta)",      2,    30,   8),
    ("tAvanzarHuecoMs",  "Override avance tras hueco ms", 0,    5000, 0),
    ("minHuecoStableMs", "Hueco estable min (ms)",        0,    2000, 250),
    # Geometria fisica del carro y parametros de la maniobra automatica.
    ("carLargoCm",       "Largo del carro (cm)",          5,    100,  29),
    ("carAnchoCm",       "Ancho del carro (cm)",          5,    50,   14),
    ("carAltoCm",        "Alto del carro (cm)",           5,    50,   25),
    ("margenHuecoCm",    "Margen lateral hueco (cm)",     0,    30,   6),
    ("margenBanquetaCm", "Margen banqueta (cm)",          0,    20,   4),
    ("cmPorSegPark",     "Velocidad parking (cm/s)",      5,    200,  30),
    ("distInicialCm",    "Avance inicial (cm)",           20,   300,  100),
    ("kickStartPWM",     "Kick-start PWM",                100,  255,  240),
    ("kickStartMs",      "Kick-start duracion (ms)",      0,    500,  150),
    ("tReversaGiroMs",   "Tiempo reversa con giro (ms)",  200,  5000, 1600),
]

STATE_COLORS = {
    "MANUAL":  "#9e9e9e",
    "TEST":    "#42a5f5",
    "AUTO":    "#ffb300",
    "DONE":    "#43a047",
    "ABORT":   "#e53935",
    "?":       "#9e9e9e",
}


class CarControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AutoModelCar - Panel de Control")
        self.root.geometry("1180x720")
        self.root.configure(bg="#1e1e1e")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".",
                        background="#1e1e1e", foreground="#e0e0e0",
                        fieldbackground="#2a2a2a")
        style.configure("TLabel", background="#1e1e1e", foreground="#e0e0e0")
        style.configure("TLabelframe", background="#1e1e1e", foreground="#e0e0e0")
        style.configure("TLabelframe.Label", background="#1e1e1e",
                        foreground="#90caf9", font=("Segoe UI", 10, "bold"))
        style.configure("TFrame", background="#1e1e1e")
        style.configure("TButton", padding=6, font=("Segoe UI", 9))
        style.configure("Big.TButton", padding=10, font=("Segoe UI", 11, "bold"))
        style.configure("Estop.TButton", padding=14,
                        font=("Segoe UI", 13, "bold"),
                        background="#e53935", foreground="white")
        style.map("Estop.TButton", background=[("active", "#ff5252")])

        self.ws = None
        self.connected = False
        self.pressed = set()

        self.slider_vars = {}
        for k, _, _, _, default in SLIDERS:
            self.slider_vars[k] = tk.IntVar(value=default)

        self._build_ui()

        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # Construye toda la interfaz: tres columnas (estado, controles,
    # calibracion). Se llama una sola vez desde __init__.
    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        outer.columnconfigure(0, weight=1, uniform="col")
        outer.columnconfigure(1, weight=1, uniform="col")
        outer.columnconfigure(2, weight=1, uniform="col")
        outer.rowconfigure(0, weight=1)

        col_left  = ttk.Frame(outer); col_left.grid (row=0, column=0, sticky="nsew", padx=4)
        col_mid   = ttk.Frame(outer); col_mid.grid  (row=0, column=1, sticky="nsew", padx=4)
        col_right = ttk.Frame(outer); col_right.grid(row=0, column=2, sticky="nsew", padx=4)

        self._build_left  (col_left)
        self._build_mid   (col_mid)
        self._build_right (col_right)

    def _build_left(self, parent):
        conn = ttk.LabelFrame(parent, text="Conexion", padding=10)
        conn.pack(fill="x", pady=(0, 8))

        self.lbl_status = ttk.Label(conn, text="Desconectado",
                                    font=("Segoe UI", 10, "bold"),
                                    foreground="#e57373")
        self.lbl_status.pack(side="left")

        ttk.Button(conn, text="Conectar",    command=self.connect   ).pack(side="right", padx=3)
        ttk.Button(conn, text="Desconectar", command=self.disconnect).pack(side="right", padx=3)

        st = ttk.LabelFrame(parent, text="Estado", padding=10)
        st.pack(fill="x", pady=(0, 8))

        self.state_canvas = tk.Canvas(st, height=70, bg="#9e9e9e",
                                      highlightthickness=0)
        self.state_canvas.pack(fill="x")
        self.state_text = self.state_canvas.create_text(
            10, 35, anchor="w", text="MANUAL",
            font=("Segoe UI", 18, "bold"), fill="white")
        self.substate_text = self.state_canvas.create_text(
            10, 58, anchor="w", text="-",
            font=("Segoe UI", 9), fill="white")

        sens = ttk.LabelFrame(parent, text="Sensores (cm)", padding=10)
        sens.pack(fill="x", pady=(0, 8))

        self.lbl_R = self._mk_big_label(sens, "Derecho",   "999")
        self.lbl_L = self._mk_big_label(sens, "Izquierdo", "999")
        self.lbl_B = self._mk_big_label(sens, "Trasero",   "999")

        dr = ttk.LabelFrame(parent, text="Direccion (estimada)", padding=10)
        dr.pack(fill="x", pady=(0, 8))

        self.lbl_steer_pos = ttk.Label(dr, text="200 ms",
                                       font=("Segoe UI", 12, "bold"))
        self.lbl_steer_pos.pack(anchor="w")

        self.steer_bar = tk.Canvas(dr, height=20, bg="#2a2a2a",
                                   highlightthickness=0)
        self.steer_bar.pack(fill="x", pady=(4, 0))
        self.steer_bar_rect = self.steer_bar.create_rectangle(
            0, 0, 0, 20, fill="#42a5f5", width=0)

    def _mk_big_label(self, parent, name, initial):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=name + ":", width=10).pack(side="left")
        lbl = ttk.Label(row, text=initial, font=("Consolas", 16, "bold"),
                        foreground="#90caf9")
        lbl.pack(side="left")
        return lbl

    def _build_mid(self, parent):
        ctrl = ttk.LabelFrame(parent, text="Control manual (WASD)", padding=10)
        ctrl.pack(fill="x", pady=(0, 8))

        row1 = ttk.Frame(ctrl); row1.pack(fill="x", pady=3)
        ttk.Button(row1, text="W\nForward", style="Big.TButton",
                   command=lambda: self.send("W")
                   ).pack(side="left", expand=True, fill="both", padx=2)
        ttk.Button(row1, text="X\nStop", style="Big.TButton",
                   command=lambda: self.send("X")
                   ).pack(side="left", expand=True, fill="both", padx=2)
        ttk.Button(row1, text="S\nReverse", style="Big.TButton",
                   command=lambda: self.send("S")
                   ).pack(side="left", expand=True, fill="both", padx=2)

        row2 = ttk.Frame(ctrl); row2.pack(fill="x", pady=3)
        ttk.Button(row2, text="A\nLeft", style="Big.TButton",
                   command=lambda: self.send("A")
                   ).pack(side="left", expand=True, fill="both", padx=2)
        ttk.Button(row2, text="C\nCenter", style="Big.TButton",
                   command=lambda: self.send("C")
                   ).pack(side="left", expand=True, fill="both", padx=2)
        ttk.Button(row2, text="D\nRight", style="Big.TButton",
                   command=lambda: self.send("D")
                   ).pack(side="left", expand=True, fill="both", padx=2)

        auto = ttk.LabelFrame(parent, text="Modo autonomo", padding=10)
        auto.pack(fill="x", pady=(0, 8))

        prow = ttk.Frame(auto); prow.pack(fill="x", pady=3)
        ttk.Button(prow, text="Estacionar IZQUIERDA", style="Big.TButton",
                   command=lambda: self.send("PL")
                   ).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(prow, text="Estacionar DERECHA", style="Big.TButton",
                   command=lambda: self.send("PR")
                   ).pack(side="left", expand=True, fill="x", padx=2)

        urow = ttk.Frame(auto); urow.pack(fill="x", pady=3)
        ttk.Button(urow, text="Sensor Test (T)",
                   command=lambda: self.send("T")
                   ).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(urow, text="Hazards (H)",
                   command=lambda: self.send("H")
                   ).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(urow, text="Recalibrar (CAL)",
                   command=lambda: self.send("CAL")
                   ).pack(side="left", expand=True, fill="x", padx=2)

        estop = ttk.Frame(parent)
        estop.pack(fill="x", pady=(8, 0))
        ttk.Button(estop, text="PARO DE EMERGENCIA",
                   style="Estop.TButton",
                   command=self.estop).pack(fill="x", ipady=6)

        info = ttk.Label(parent,
                         text="Tip: WASD con la ventana enfocada.\n"
                              "Cualquier tecla durante AUTO cancela.\n"
                              "Esc = paro de emergencia.",
                         justify="left", foreground="#9e9e9e")
        info.pack(anchor="w", pady=(8, 0))

    def _build_right(self, parent):
        cal = ttk.LabelFrame(parent, text="Calibracion en vivo", padding=10)
        cal.pack(fill="both", expand=True)

        canvas = tk.Canvas(cal, bg="#1e1e1e", highlightthickness=0)
        sb = ttk.Scrollbar(cal, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)

        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)

        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        for key, label, mn, mx, default in SLIDERS:
            self._mk_slider(inner, key, label, mn, mx, default)

    def _mk_slider(self, parent, key, label, mn, mx, default):
        frm = ttk.Frame(parent)
        frm.pack(fill="x", pady=4)

        head = ttk.Frame(frm); head.pack(fill="x")
        ttk.Label(head, text=label, font=("Segoe UI", 9)).pack(side="left")
        val_lbl = ttk.Label(head, text=str(default),
                            font=("Consolas", 10, "bold"),
                            foreground="#90caf9", width=6, anchor="e")
        val_lbl.pack(side="right")

        var = self.slider_vars[key]

        def on_change(_e=None):
            v = int(float(var.get()))
            val_lbl.config(text=str(v))
            self.send(f"SET:{key}={v}")

        sc = ttk.Scale(frm, from_=mn, to=mx, orient="horizontal",
                       variable=var, command=on_change)
        sc.pack(fill="x")

    # Loop de asyncio que vive en su propio thread. Toda la
    # comunicacion WebSocket se programa con run_coroutine_threadsafe
    # para no bloquear el thread principal de Tkinter.
    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def connect(self):
        asyncio.run_coroutine_threadsafe(self._connect_ws(), self.loop)

    async def _connect_ws(self):
        try:
            if self.ws:
                await self.ws.close()
            self.ws = await websockets.connect(WS_URL)
            self.connected = True
            self._set_status(True)
            asyncio.ensure_future(self._listen_loop())
        except Exception as e:
            self.connected = False
            self.ws = None
            self._set_status(False)
            self.root.after(0, lambda: messagebox.showerror(
                "Error", f"No se pudo conectar:\n{e}"))

    async def _listen_loop(self):
        try:
            while self.ws:
                msg = await self.ws.recv()
                self._handle_telemetry(msg)
        except Exception:
            self.ws = None
            self.connected = False
            self._set_status(False)

    def disconnect(self):
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

    # Helpers de UI: actualizan widgets desde callbacks de Tkinter.
    # Todo lo que toca widgets se enrutea por root.after para que
    # corra en el thread principal aunque venga del listener async.
    def _set_status(self, ok: bool):
        def _do():
            if ok:
                self.lbl_status.config(text="Conectado", foreground="#81c784")
            else:
                self.lbl_status.config(text="Desconectado", foreground="#e57373")
        self.root.after(0, _do)

    def _handle_telemetry(self, msg):
        try:
            j = json.loads(msg)
        except Exception:
            return
        if j.get("t") != "tel":
            return

        def _do():
            self.lbl_R.config(text=str(j.get("R", "?")))
            self.lbl_L.config(text=str(j.get("L", "?")))
            self.lbl_B.config(text=str(j.get("B", "?")))

            sp = int(j.get("sp", 0))
            self.lbl_steer_pos.config(text=f"{sp} ms")

            tmax = self.slider_vars["tSteerFullMs"].get() or 400
            ratio = max(0.0, min(1.0, sp / float(tmax)))
            w = self.steer_bar.winfo_width()
            self.steer_bar.coords(self.steer_bar_rect, 0, 0, w * ratio, 20)

            mode = j.get("mode", "?")
            sub  = j.get("st", "-")
            color = STATE_COLORS.get(mode, "#9e9e9e")
            self.state_canvas.config(bg=color)
            self.state_canvas.itemconfig(self.state_text, text=mode)
            self.state_canvas.itemconfig(self.substate_text, text=sub)

        self.root.after(0, _do)

    def estop(self):
        self.send("ESTOP")

    # Manejo de teclado. WASD se mandan como comandos individuales
    # al firmware (igual que los botones), y al soltar la tecla se
    # envia "X" (parar tracion) o "C" (centrar direccion) segun el caso.
    def on_key_press(self, event):
        k = event.keysym.lower()
        if k in self.pressed:
            return
        self.pressed.add(k)

        if k == "w":      self.send("W")
        elif k == "s":    self.send("S")
        elif k == "a":    self.send("A")
        elif k == "d":    self.send("D")
        elif k == "escape": self.estop()

    def on_key_release(self, event):
        k = event.keysym.lower()
        self.pressed.discard(k)

        if k in ("w", "s"):
            self.send("X")
        if k in ("a", "d"):
            self.send("C")

    def on_close(self):
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