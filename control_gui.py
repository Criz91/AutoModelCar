import json
import socket
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# Conexion al ESP32 (puerto TCP que abre el firmware unificado).
# El reglamento del TMR prohibe comunicacion cableada con procesamiento
# externo, asi que la GUI siempre se conecta por WiFi al AP del ESP.
DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 8080


# Lista de sliders de calibracion que aparecen en el panel derecho.
# Cada entrada es (clave, etiqueta visible, minimo, maximo, valor inicial).
# La clave es la misma que entiende el firmware en applyParam(), asi al
# mover el slider se manda "SET:clave=valor" por TCP y el ESP32 ajusta
# el parametro en vivo sin reflashear.
SLIDERS = [
    ("driveSpeed",       "Velocidad manual (PWM)",        0,    255,  200),
    ("parkDriveSpeed",   "Velocidad estacionar (PWM)",    0,    255,  200),
    ("lineFollowSpeed",  "Velocidad line follow (PWM)",   0,    255,  150),
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
    "LF":      "#26c6da",
    "TPARK":   "#ff9800",
    "DONE":    "#43a047",
    "ABORT":   "#e53935",
    "?":       "#9e9e9e",
}


class CarControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AutoModelCar - Panel de Control (TCP)")
        self.root.geometry("1180x760")
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
        style.configure("TEntry", fieldbackground="#2a2a2a", foreground="#e0e0e0")
        style.configure("TButton", padding=6, font=("Segoe UI", 9))
        style.configure("Big.TButton", padding=10, font=("Segoe UI", 11, "bold"))
        style.configure("Estop.TButton", padding=14,
                        font=("Segoe UI", 13, "bold"),
                        background="#e53935", foreground="white")
        style.map("Estop.TButton", background=[("active", "#ff5252")])

        # Estado del socket TCP. self.sock vive en el thread del listener;
        # las escrituras se hacen desde el thread de Tkinter usando un lock.
        self.sock = None
        self.sock_lock = threading.Lock()
        self.connected = False
        self.listener_thread = None
        self.stop_listener = threading.Event()

        self.pressed = set()

        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        self.port_var = tk.StringVar(value=str(DEFAULT_PORT))

        self.slider_vars = {}
        for k, _, _, _, default in SLIDERS:
            self.slider_vars[k] = tk.IntVar(value=default)

        self._build_ui()

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
        conn = ttk.LabelFrame(parent, text="Conexion (TCP)", padding=10)
        conn.pack(fill="x", pady=(0, 8))

        # Fila 1: host:port editables
        addr_row = ttk.Frame(conn); addr_row.pack(fill="x")
        ttk.Label(addr_row, text="Host:", width=6).pack(side="left")
        ttk.Entry(addr_row, textvariable=self.host_var, width=14).pack(side="left", padx=(0,6))
        ttk.Label(addr_row, text="Puerto:").pack(side="left")
        ttk.Entry(addr_row, textvariable=self.port_var, width=6).pack(side="left", padx=(4,0))

        # Fila 2: status + botones
        st_row = ttk.Frame(conn); st_row.pack(fill="x", pady=(8,0))
        self.lbl_status = ttk.Label(st_row, text="Desconectado",
                                    font=("Segoe UI", 10, "bold"),
                                    foreground="#e57373")
        self.lbl_status.pack(side="left")

        ttk.Button(st_row, text="Conectar",    command=self.connect   ).pack(side="right", padx=3)
        ttk.Button(st_row, text="Desconectar", command=self.disconnect).pack(side="right", padx=3)

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

        sens = ttk.LabelFrame(parent, text="Ultrasonicos (cm)", padding=10)
        sens.pack(fill="x", pady=(0, 8))

        self.lbl_R = self._mk_big_label(sens, "Derecho",   "999")
        self.lbl_L = self._mk_big_label(sens, "Izquierdo", "999")
        self.lbl_B = self._mk_big_label(sens, "Trasero",   "999")
        self.lbl_F = self._mk_big_label(sens, "Frontal",   "999")

        line = ttk.LabelFrame(parent, text="Seguidores de linea (TCRT5000)", padding=10)
        line.pack(fill="x", pady=(0, 8))
        self.line_canvas = tk.Canvas(line, height=46, bg="#1e1e1e",
                                     highlightthickness=0)
        self.line_canvas.pack(fill="x")
        # Tres circulos: izquierdo, centro, derecho
        self.line_dots = []
        labels = ["L", "C", "R"]
        for i, lab in enumerate(labels):
            x = 40 + i * 80
            dot = self.line_canvas.create_oval(x-15, 8, x+15, 38,
                                               fill="#424242", outline="#90caf9", width=2)
            self.line_canvas.create_text(x, 23, text=lab,
                                         font=("Segoe UI", 11, "bold"),
                                         fill="#ffffff")
            self.line_dots.append(dot)

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

        tprow = ttk.Frame(auto); tprow.pack(fill="x", pady=3)
        ttk.Button(tprow, text="Test Park IZQ (ciego)",
                   command=lambda: self.send("TPARKL")
                   ).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(tprow, text="Test Park DER (ciego)",
                   command=lambda: self.send("TPARKR")
                   ).pack(side="left", expand=True, fill="x", padx=2)

        lfrow = ttk.Frame(auto); lfrow.pack(fill="x", pady=3)
        ttk.Button(lfrow, text="Line Follow ON (LF)", style="Big.TButton",
                   command=lambda: self.send("LF")
                   ).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(lfrow, text="Line Follow OFF (NOLF)", style="Big.TButton",
                   command=lambda: self.send("NOLF")
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
                              "Cualquier tecla durante AUTO o LF cancela.\n"
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

    # Conexion TCP
    # El listener vive en un thread dedicado bloqueante. Lee lineas
    # delimitadas por '\n' del socket y las pasa al handler. El thread
    # de Tkinter solo escribe (con lock) y se actualiza via root.after.
    def connect(self):
        if self.connected:
            return
        host = self.host_var.get().strip() or DEFAULT_HOST
        try:
            port = int(self.port_var.get())
        except ValueError:
            port = DEFAULT_PORT

        try:
            s = socket.create_connection((host, port), timeout=3)
            s.settimeout(None)  # bloqueante despues del connect
        except Exception as e:
            self._set_status(False)
            messagebox.showerror("Error", f"No se pudo conectar a {host}:{port}\n{e}")
            return

        with self.sock_lock:
            self.sock = s
        self.connected = True
        self._set_status(True)

        self.stop_listener.clear()
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()

    def disconnect(self):
        self.stop_listener.set()
        with self.sock_lock:
            s = self.sock
            self.sock = None
        if s:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                s.close()
            except Exception:
                pass
        self.connected = False
        self._set_status(False)

    def _listen_loop(self):
        buf = b""
        while not self.stop_listener.is_set():
            with self.sock_lock:
                s = self.sock
            if s is None:
                break
            try:
                chunk = s.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    text = line.decode("utf-8", errors="replace")
                except Exception:
                    continue
                self._handle_message(text)

        # Cerro el remoto o hubo error
        self.connected = False
        with self.sock_lock:
            self.sock = None
        self._set_status(False)

    def send(self, msg: str):
        with self.sock_lock:
            s = self.sock
        if s is None:
            return
        try:
            s.sendall((msg + "\n").encode("utf-8"))
        except OSError:
            self.connected = False
            with self.sock_lock:
                self.sock = None
            self._set_status(False)

    # Helpers de UI: actualizan widgets desde callbacks de Tkinter.
    # Todo lo que toca widgets se enrutea por root.after para que
    # corra en el thread principal aunque venga del listener.
    def _set_status(self, ok: bool):
        def _do():
            if ok:
                self.lbl_status.config(text="Conectado", foreground="#81c784")
            else:
                self.lbl_status.config(text="Desconectado", foreground="#e57373")
        self.root.after(0, _do)

    def _handle_message(self, msg):
        # El firmware manda telemetria como JSON y otras lineas como
        # texto plano (RX,..., INFO,..., ERR,...). Solo nos interesa
        # parsear las que empiezan con '{' como JSON.
        if not msg.startswith("{"):
            return
        try:
            j = json.loads(msg)
        except Exception:
            return
        if j.get("t") != "tel":
            return
        self._handle_telemetry(j)

    def _handle_telemetry(self, j):
        def _do():
            self.lbl_R.config(text=str(j.get("R", "?")))
            self.lbl_L.config(text=str(j.get("L", "?")))
            self.lbl_B.config(text=str(j.get("B", "?")))
            self.lbl_F.config(text=str(j.get("F", "?")))

            # Seguidores de linea: tres circulos. Verde = ve linea blanca.
            colors = []
            for key in ("lnL", "lnC", "lnR"):
                colors.append("#66bb6a" if int(j.get(key, 0)) else "#424242")
            for dot, col in zip(self.line_dots, colors):
                self.line_canvas.itemconfig(dot, fill=col)

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

    # Manejo de teclado
    # WASD se mandan como comandos individuales al firmware (igual
    # que los botones), y al soltar la tecla se envia "X" (parar
    # tracion) o "C" (centrar direccion) segun el caso.
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
