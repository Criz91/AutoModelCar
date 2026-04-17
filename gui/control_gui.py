import tkinter as tk
from tkinter import ttk
import socket
import threading
import queue
import json
import time

HOST_DEFAULT  = "192.168.4.1"
PUERTO_DEFAULT = 8080

# Colores
COLOR_FONDO        = "#1e1e2e"
COLOR_PANEL        = "#2a2a3e"
COLOR_BORDE        = "#3a3a5e"
COLOR_TEXTO        = "#cdd6f4"
COLOR_TEXTO_DIM    = "#6c7086"
COLOR_VERDE        = "#a6e3a1"
COLOR_ROJO         = "#f38ba8"
COLOR_AZUL         = "#89b4fa"
COLOR_AMARILLO     = "#f9e2af"
COLOR_NARANJA      = "#fab387"
COLOR_MORADO       = "#cba6f7"
COLOR_BOTON        = "#313244"
COLOR_BOTON_HOVER  = "#45475a"

FUENTE_TITULO  = ("Segoe UI", 11, "bold")
FUENTE_NORMAL  = ("Segoe UI", 9)
FUENTE_MONO    = ("Consolas", 8)
FUENTE_GRANDE  = ("Segoe UI", 13, "bold")
FUENTE_NUMERO  = ("Consolas", 11, "bold")

# Sliders: (clave, label, min, max, default)
SLIDERS = [
    ("velocidadParking",    "Vel. parking",       50,  255, 220),
    ("velocidadAvance",     "Vel. avance",         50,  255, 220),
    ("velocidadDireccion",  "Vel. direccion",      50,  255, 208),
    ("tiempoDireccionTope", "Tope direccion (ms)", 100, 800, 355),
    ("tAvance1Ms",          "tAvance1 (ms)",       500, 5000, 2000),
    ("tAvance2Ms",          "tAvance2 (ms)",       500, 5000, 2000),
    ("tGiroIzqMs",          "tGiroIzq (ms)",       200, 3000, 1000),
    ("tReversaGiroMs",      "tReversaGiro (ms)",   500, 5000, 2000),
    ("tReversaRectaMs",     "tReversaRecta (ms)",  500, 8000, 4000),
    ("distCarroCm",         "Dist. carro (cm)",      5,   40,   15),
    ("distBanquetaCm",      "Dist. banqueta (cm)",   2,   20,    5),
]


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AutoModelCar - Control TMR 2026")
        self.configure(bg=COLOR_FONDO)
        self.resizable(True, True)
        self.minsize(1100, 700)

        self.sock   = None
        self.cola   = queue.Queue()
        self.hilo   = None
        self.conectado = False

        self._construir_ui()
        self._drenar_cola()
        
        # Trackers para no inundar el log
        self.ultimo_modo = None
        self.ultimo_estado = None

        # Bindings de teclado (Bug 4) con control Anti-Tartamudeo
        self.teclas_presionadas = {}
        self.teclas_timers = {} 
        self.bind_all('<KeyPress>', self._on_tecla_press)
        self.bind_all('<KeyRelease>', self._on_tecla_release)

    def _enviar_continuo(self, tecla):
        # Latido: reenvia el comando cada 500ms mientras se sostenga la tecla
        if self.teclas_presionadas.get(tecla, False):
            if tecla in ["W", "A", "S", "D"]:
                self.enviar(tecla)
            self.after(500, lambda: self._enviar_continuo(tecla))

    def _on_tecla_press(self, event):
        if isinstance(self.focus_get(), tk.Entry): return
        tecla = event.char.upper()
        
        if tecla in self.teclas_timers:
            self.after_cancel(self.teclas_timers[tecla])
            del self.teclas_timers[tecla]
            
        if not self.teclas_presionadas.get(tecla, False):
            self.teclas_presionadas[tecla] = True
            if tecla in ["W", "A", "S", "D"]:
                self.enviar(tecla)
                self.after(500, lambda: self._enviar_continuo(tecla)) # Inicia el latido
            elif event.keysym == "space":
                self.enviar("X")

    def _on_tecla_release(self, event):
        if isinstance(self.focus_get(), tk.Entry): return
        tecla = event.char.upper()
        
        timer = self.after(50, lambda: self._ejecutar_release(tecla))
        self.teclas_timers[tecla] = timer

    def _ejecutar_release(self, tecla):
        self.teclas_presionadas[tecla] = False
        if tecla in self.teclas_timers:
            del self.teclas_timers[tecla]
            
        # Al soltar, mandar el comando de detener correspondiente
        if tecla in ["W", "S"]:
            self.enviar("X") # Frena traccion
        elif tecla in ["A", "D"]:
            self.enviar("C") # Frena direccion

    # ── Construccion de la UI ─────────────────────────────────────────────────

    def _construir_ui(self):
        self._barra_conexion()

        contenedor = tk.Frame(self, bg=COLOR_FONDO)
        contenedor.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        contenedor.grid_columnconfigure(0, weight=0, minsize=270)
        contenedor.grid_columnconfigure(1, weight=1, minsize=380)
        contenedor.grid_columnconfigure(2, weight=0, minsize=390)
        contenedor.grid_rowconfigure(0, weight=1)

        self._panel_control(contenedor)
        self._panel_monitor(contenedor)
        self._panel_parametros(contenedor)

    def _barra_conexion(self):
        barra = tk.Frame(self, bg=COLOR_PANEL, pady=6)
        barra.pack(fill=tk.X, padx=8, pady=(8, 4))

        tk.Label(barra, text="HOST", bg=COLOR_PANEL, fg=COLOR_TEXTO_DIM,
                 font=FUENTE_NORMAL).pack(side=tk.LEFT, padx=(10, 2))
        self.entrada_host = tk.Entry(barra, width=14, bg=COLOR_BOTON,
                                     fg=COLOR_TEXTO, insertbackground=COLOR_TEXTO,
                                     relief=tk.FLAT, font=FUENTE_NORMAL)
        self.entrada_host.insert(0, HOST_DEFAULT)
        self.entrada_host.pack(side=tk.LEFT, padx=(0, 8))

        tk.Label(barra, text="PUERTO", bg=COLOR_PANEL, fg=COLOR_TEXTO_DIM,
                 font=FUENTE_NORMAL).pack(side=tk.LEFT, padx=(0, 2))
        self.entrada_puerto = tk.Entry(barra, width=6, bg=COLOR_BOTON,
                                       fg=COLOR_TEXTO, insertbackground=COLOR_TEXTO,
                                       relief=tk.FLAT, font=FUENTE_NORMAL)
        self.entrada_puerto.insert(0, str(PUERTO_DEFAULT))
        self.entrada_puerto.pack(side=tk.LEFT, padx=(0, 12))

        self.btn_conectar = tk.Button(barra, text="Conectar", width=10,
                                      bg=COLOR_VERDE, fg=COLOR_FONDO,
                                      activebackground=COLOR_VERDE,
                                      font=("Segoe UI", 9, "bold"),
                                      relief=tk.FLAT, cursor="hand2",
                                      command=self._conectar)
        self.btn_conectar.pack(side=tk.LEFT, padx=(0, 4))

        self.btn_desconectar = tk.Button(barra, text="Desconectar", width=12,
                                         bg=COLOR_BOTON, fg=COLOR_TEXTO_DIM,
                                         activebackground=COLOR_BOTON_HOVER,
                                         font=FUENTE_NORMAL, relief=tk.FLAT,
                                         cursor="hand2",
                                         command=self._desconectar,
                                         state=tk.DISABLED)
        self.btn_desconectar.pack(side=tk.LEFT, padx=(0, 12))

        # Indicador de conexion
        self.canvas_estado_con = tk.Canvas(barra, width=14, height=14,
                                           bg=COLOR_PANEL, highlightthickness=0)
        self.canvas_estado_con.pack(side=tk.LEFT, padx=(0, 4))
        self.circulo_con = self.canvas_estado_con.create_oval(2, 2, 12, 12,
                                                               fill=COLOR_ROJO, outline="")

        self.label_estado_con = tk.Label(barra, text="Desconectado",
                                         bg=COLOR_PANEL, fg=COLOR_ROJO,
                                         font=FUENTE_NORMAL)
        self.label_estado_con.pack(side=tk.LEFT)

    def _panel_control(self, padre):
        frame = tk.Frame(padre, bg=COLOR_FONDO)
        frame.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 6))

        self._grupo_wasd(frame)
        self._grupo_rutinas(frame)
        self._grupo_leds(frame)

    def _grupo_wasd(self, padre):
        grp = self._labelframe(padre, "Control manual")
        grp.pack(fill=tk.X, pady=(0, 6))

        nota = tk.Label(grp, text="A / D: solo mueven la direccion\nW / S + A / D = giro avanzando",
                        bg=COLOR_PANEL, fg=COLOR_TEXTO_DIM, font=("Segoe UI", 8),
                        justify=tk.LEFT)
        nota.pack(anchor=tk.W, pady=(0, 6))

        cruz = tk.Frame(grp, bg=COLOR_PANEL)
        cruz.pack()

        self._btn_ctrl(cruz, "W\nAvanzar",  0, 1, "W")
        self._btn_ctrl(cruz, "A\nDir Izq",  1, 0, "A")
        self._btn_ctrl(cruz, "X\nFreno",    1, 1, "X", color=COLOR_AMARILLO)
        self._btn_ctrl(cruz, "D\nDir Der",  1, 2, "D")
        self._btn_ctrl(cruz, "C\nDir Stop", 2, 1, "C", color=COLOR_TEXTO_DIM)
        self._btn_ctrl(cruz, "S\nReversa",  3, 1, "S")
        # Botones de Test de Topes
        test_frame = tk.Frame(grp, bg=COLOR_PANEL)
        test_frame.pack(fill=tk.X, pady=(10, 0))
        
        tk.Button(test_frame, text="TEST TOPE IZQ", bg=COLOR_BOTON, fg=COLOR_AMARILLO,
                  font=("Segoe UI", 8, "bold"), cursor="hand2", relief=tk.FLAT,
                  command=lambda: self.enviar("TEST_DIR:IZQ")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
                  
        tk.Button(test_frame, text="TEST TOPE DER", bg=COLOR_BOTON, fg=COLOR_AMARILLO,
                  font=("Segoe UI", 8, "bold"), cursor="hand2", relief=tk.FLAT,
                  command=lambda: self.enviar("TEST_DIR:DER")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

    def _btn_ctrl(self, padre, texto, fila, col, cmd, color=None):
        color = color or COLOR_AZUL
        b = tk.Button(padre, text=texto, width=7, height=2,
                      bg=COLOR_BOTON, fg=color,
                      activebackground=COLOR_BOTON_HOVER, activeforeground=color,
                      font=("Segoe UI", 8, "bold"), relief=tk.FLAT, cursor="hand2",
                      command=lambda: self.enviar(cmd))
        b.grid(row=fila, column=col, padx=2, pady=2)

    def _grupo_rutinas(self, padre):
        grp = self._labelframe(padre, "Rutinas autonomas")
        grp.pack(fill=tk.X, pady=(0, 6))

        self.btn_tc = tk.Button(grp, text="TEST CIEGO", height=2,
                                bg="#2d4a2d", fg=COLOR_VERDE,
                                activebackground="#3d6a3d", activeforeground=COLOR_VERDE,
                                font=("Segoe UI", 10, "bold"), relief=tk.FLAT,
                                cursor="hand2", command=lambda: self.enviar("TC"))
        self.btn_tc.pack(fill=tk.X, pady=(0, 4))

        self.btn_pr = tk.Button(grp, text="ESTACIONAR DERECHA", height=2,
                                bg="#1e2d4a", fg=COLOR_AZUL,
                                activebackground="#2e3d6a", activeforeground=COLOR_AZUL,
                                font=("Segoe UI", 10, "bold"), relief=tk.FLAT,
                                cursor="hand2", command=lambda: self.enviar("PR"))
        self.btn_pr.pack(fill=tk.X, pady=(0, 4))

        self.btn_stop = tk.Button(grp, text="STOP", height=3,
                                  bg="#4a1e1e", fg=COLOR_ROJO,
                                  activebackground="#6a2e2e", activeforeground=COLOR_ROJO,
                                  font=("Segoe UI", 13, "bold"), relief=tk.FLAT,
                                  cursor="hand2", command=lambda: self.enviar("STOP"))
        self.btn_stop.pack(fill=tk.X)

    def _grupo_leds(self, padre):
        grp = self._labelframe(padre, "Prueba de LEDs")
        grp.pack(fill=tk.X, pady=(0, 6))

        fila1 = tk.Frame(grp, bg=COLOR_PANEL)
        fila1.pack(fill=tk.X, pady=(0, 3))
        fila2 = tk.Frame(grp, bg=COLOR_PANEL)
        fila2.pack(fill=tk.X)

        botones_led = [
            ("Freno ON",  "LED:FRENO:1", fila1),
            ("Freno OFF", "LED:FRENO:0", fila1),
            ("Inter IZQ", "LED:INTER:IZQ", fila2),
            ("Inter DER", "LED:INTER:DER", fila2),
            ("Inter AMBAS","LED:INTER:AMBAS", fila2),
            ("LEDs OFF",  "LED:OFF", fila2),
        ]
        for texto, cmd, contenedor in botones_led:
            tk.Button(contenedor, text=texto,
                      bg=COLOR_BOTON, fg=COLOR_TEXTO,
                      activebackground=COLOR_BOTON_HOVER,
                      font=("Segoe UI", 8), relief=tk.FLAT, cursor="hand2",
                      command=lambda c=cmd: self.enviar(c)).pack(
                side=tk.LEFT, padx=2, pady=1, expand=True, fill=tk.X)

    def _panel_monitor(self, padre):
        frame = tk.Frame(padre, bg=COLOR_FONDO)
        frame.grid(row=0, column=1, sticky=tk.NSEW, padx=(0, 6))

        self._grupo_estado(frame)
        self._grupo_sensores(frame)
        self._grupo_actuadores(frame)

    def _grupo_estado(self, padre):
        grp = self._labelframe(padre, "Estado del vehiculo")
        grp.pack(fill=tk.X, pady=(0, 6))

        self.label_modo = tk.Label(grp, text="Modo:   MANUAL",
                                   bg=COLOR_PANEL, fg=COLOR_AZUL,
                                   font=FUENTE_GRANDE, anchor=tk.W)
        self.label_modo.pack(fill=tk.X)

        self.label_estado = tk.Label(grp, text="Estado: NINGUNO",
                                     bg=COLOR_PANEL, fg=COLOR_TEXTO,
                                     font=FUENTE_TITULO, anchor=tk.W)
        self.label_estado.pack(fill=tk.X)

        self.label_tiempo = tk.Label(grp, text="En estado: 0 ms",
                                     bg=COLOR_PANEL, fg=COLOR_TEXTO_DIM,
                                     font=FUENTE_NORMAL, anchor=tk.W)
        self.label_tiempo.pack(fill=tk.X)

        self.label_hueco = tk.Label(grp, text="Hueco medido: 0.0 cm",
                                    bg=COLOR_PANEL, fg=COLOR_TEXTO_DIM,
                                    font=FUENTE_NORMAL, anchor=tk.W)
        self.label_hueco.pack(fill=tk.X)

    def _grupo_sensores(self, padre):
        grp = self._labelframe(padre, "Ultrasonicos (cm)")
        grp.pack(fill=tk.X, pady=(0, 6))

        self.barras_sensor = {}
        self.labels_sensor = {}

        sensores = [
            ("dR", "Derecho  "),
            ("dL", "Izquierdo"),
            ("dB", "Trasero  "),
            ("dF", "Frontal  "),
        ]
        for clave, nombre in sensores:
            fila = tk.Frame(grp, bg=COLOR_PANEL)
            fila.pack(fill=tk.X, pady=2)

            tk.Label(fila, text=nombre, bg=COLOR_PANEL, fg=COLOR_TEXTO_DIM,
                     font=FUENTE_MONO, width=10, anchor=tk.W).pack(side=tk.LEFT)

            canvas = tk.Canvas(fila, height=16, bg=COLOR_BOTON,
                               highlightthickness=0)
            canvas.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 4))
            barra = canvas.create_rectangle(0, 0, 0, 16, fill=COLOR_VERDE, outline="")
            self.barras_sensor[clave] = (canvas, barra)

            lbl = tk.Label(fila, text="--- cm", bg=COLOR_PANEL, fg=COLOR_VERDE,
                           font=FUENTE_NUMERO, width=8, anchor=tk.E)
            lbl.pack(side=tk.LEFT)
            self.labels_sensor[clave] = lbl

    def _grupo_actuadores(self, padre):
        grp = self._labelframe(padre, "Actuadores y LEDs")
        grp.pack(fill=tk.X, pady=(0, 6))

        # Traccion
        fila_trac = tk.Frame(grp, bg=COLOR_PANEL)
        fila_trac.pack(fill=tk.X, pady=2)
        tk.Label(fila_trac, text="Traccion  ", bg=COLOR_PANEL, fg=COLOR_TEXTO_DIM,
                 font=FUENTE_MONO, width=10, anchor=tk.W).pack(side=tk.LEFT)
        self.canvas_trac = tk.Canvas(fila_trac, height=16, bg=COLOR_BOTON,
                                     highlightthickness=0)
        self.canvas_trac.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 4))
        self.barra_trac = self.canvas_trac.create_rectangle(0, 0, 0, 16,
                                                             fill=COLOR_MORADO, outline="")
        self.label_trac = tk.Label(fila_trac, text="0 PWM", bg=COLOR_PANEL,
                                   fg=COLOR_MORADO, font=FUENTE_NUMERO, width=8, anchor=tk.E)
        self.label_trac.pack(side=tk.LEFT)

        # Posicion direccion
        fila_dir = tk.Frame(grp, bg=COLOR_PANEL)
        fila_dir.pack(fill=tk.X, pady=2)
        tk.Label(fila_dir, text="Direccion ", bg=COLOR_PANEL, fg=COLOR_TEXTO_DIM,
                 font=FUENTE_MONO, width=10, anchor=tk.W).pack(side=tk.LEFT)
        self.canvas_dir = tk.Canvas(fila_dir, height=16, bg=COLOR_BOTON,
                                    highlightthickness=0)
        self.canvas_dir.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 4))
        self.barra_dir = self.canvas_dir.create_rectangle(0, 0, 0, 16,
                                                          fill=COLOR_NARANJA, outline="")
        # Marcador de centro
        self.canvas_dir.bind("<Configure>", self._redibujar_centro_dir)
        self.linea_centro = None
        self.label_dir = tk.Label(fila_dir, text="0 ms", bg=COLOR_PANEL,
                                  fg=COLOR_NARANJA, font=FUENTE_NUMERO, width=8, anchor=tk.E)
        self.label_dir.pack(side=tk.LEFT)

        # LEDs
        fila_led = tk.Frame(grp, bg=COLOR_PANEL)
        fila_led.pack(fill=tk.X, pady=(6, 2))

        for texto, atributo, color_on in [
            ("Freno", "circulo_freno", COLOR_ROJO),
            ("Inter IZQ", "circulo_inter_izq", COLOR_AMARILLO),
            ("Inter DER", "circulo_inter_der", COLOR_AMARILLO),
        ]:
            sub = tk.Frame(fila_led, bg=COLOR_PANEL)
            sub.pack(side=tk.LEFT, padx=8)
            c = tk.Canvas(sub, width=18, height=18, bg=COLOR_PANEL, highlightthickness=0)
            c.pack()
            circulo = c.create_oval(2, 2, 16, 16, fill=COLOR_BOTON, outline=COLOR_BORDE)
            setattr(self, atributo + "_canvas", c)
            setattr(self, atributo, circulo)
            setattr(self, atributo + "_color", color_on)
            tk.Label(sub, text=texto, bg=COLOR_PANEL, fg=COLOR_TEXTO_DIM,
                     font=("Segoe UI", 7)).pack()

    def _redibujar_centro_dir(self, event):
        w = self.canvas_dir.winfo_width()
        mid = w // 2
        if self.linea_centro:
            self.canvas_dir.delete(self.linea_centro)
        self.linea_centro = self.canvas_dir.create_line(mid, 0, mid, 16,
                                                         fill=COLOR_TEXTO_DIM, width=1)

    def _panel_parametros(self, padre):
        frame = tk.Frame(padre, bg=COLOR_FONDO)
        frame.grid(row=0, column=2, sticky=tk.NSEW)
        frame.grid_rowconfigure(0, weight=0)
        frame.grid_rowconfigure(1, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        self._grupo_sliders(frame)
        self._grupo_log(frame)

    def _grupo_sliders(self, padre):
        grp = self._labelframe(padre, "Parametros de calibracion")
        grp.grid(row=0, column=0, sticky=tk.EW, pady=(0, 6))

        self.vars_slider = {}
        for clave, label, minv, maxv, defv in SLIDERS:
            fila = tk.Frame(grp, bg=COLOR_PANEL)
            fila.pack(fill=tk.X, pady=1)
            tk.Label(fila, text=label, bg=COLOR_PANEL, fg=COLOR_TEXTO,
                     font=FUENTE_NORMAL, width=20, anchor=tk.W).pack(side=tk.LEFT)
            # DoubleVar para el slider; StringVar separado para mostrar entero limpio
            var      = tk.DoubleVar(value=defv)
            disp_var = tk.StringVar(value=str(defv))
            self.vars_slider[clave] = var
            lbl_val = tk.Label(fila, textvariable=disp_var, bg=COLOR_PANEL,
                               fg=COLOR_AMARILLO, font=FUENTE_MONO, width=5)
            lbl_val.pack(side=tk.RIGHT)
            slider = ttk.Scale(fila, from_=minv, to=maxv, orient=tk.HORIZONTAL,
                               variable=var,
                               command=lambda v, dv=disp_var: dv.set(str(int(float(v)))))
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
            slider.bind("<ButtonRelease-1>",
                        lambda e, k=clave, v=var: self.enviar(f"SET:{k}={int(v.get())}"))

    def _grupo_log(self, padre):
        grp = self._labelframe(padre, "Log de comunicacion")
        grp.grid(row=1, column=0, sticky=tk.NSEW)
        grp.grid_rowconfigure(0, weight=1)
        grp.grid_columnconfigure(0, weight=1)

        self.texto_log = tk.Text(grp, bg=COLOR_BOTON, fg=COLOR_TEXTO,
                                 font=FUENTE_MONO, state=tk.DISABLED,
                                 relief=tk.FLAT, wrap=tk.NONE, height=12)
        scroll_v = tk.Scrollbar(grp, command=self.texto_log.yview,
                                bg=COLOR_BOTON, troughcolor=COLOR_FONDO)
        self.texto_log.configure(yscrollcommand=scroll_v.set)
        self.texto_log.grid(row=0, column=0, sticky=tk.NSEW)
        scroll_v.grid(row=0, column=1, sticky=tk.NS)
        grp.grid_rowconfigure(0, weight=1)
        grp.grid_columnconfigure(0, weight=1)

        self.texto_log.tag_configure("enviado",   foreground=COLOR_AZUL)
        self.texto_log.tag_configure("recibido",  foreground=COLOR_TEXTO_DIM)
        self.texto_log.tag_configure("evento",    foreground=COLOR_VERDE)
        self.texto_log.tag_configure("error",     foreground=COLOR_ROJO)

    # ── Utilidades UI ─────────────────────────────────────────────────────────

    def _labelframe(self, padre, titulo):
        frame = tk.LabelFrame(padre, text=titulo, bg=COLOR_PANEL,
                              fg=COLOR_TEXTO_DIM, font=FUENTE_NORMAL,
                              relief=tk.FLAT, bd=1,
                              highlightbackground=COLOR_BORDE,
                              highlightthickness=1, padx=8, pady=6)
        return frame

    def _log(self, texto, tag="recibido"):
        def _insertar():
            self.texto_log.configure(state=tk.NORMAL)
            ts = time.strftime("%H:%M:%S")
            self.texto_log.insert(tk.END, f"[{ts}] {texto}\n", tag)
            # Mantener maximo 200 lineas
            lineas = int(self.texto_log.index(tk.END).split(".")[0])
            if lineas > 202:
                self.texto_log.delete("1.0", f"{lineas - 200}.0")
            self.texto_log.see(tk.END)
            self.texto_log.configure(state=tk.DISABLED)
        self.after(0, _insertar)

    # ── Conexion TCP ──────────────────────────────────────────────────────────

    def _conectar(self):
        host   = self.entrada_host.get().strip()
        puerto = int(self.entrada_puerto.get().strip())
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(4)
            self.sock.connect((host, puerto))
            self.sock.settimeout(None)
            self.conectado = True
            self._actualizar_estado_conexion(True)
            self._log(f"Conectado a {host}:{puerto}", "evento")
            self.hilo = threading.Thread(target=self._leer_socket, daemon=True)
            self.hilo.start()
        except Exception as ex:
            self._log(f"Error al conectar: {ex}", "error")

    def _desconectar(self):
        self.conectado = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self._actualizar_estado_conexion(False)
        self._log("Desconectado", "evento")

    def _actualizar_estado_conexion(self, ok):
        if ok:
            self.canvas_estado_con.itemconfigure(self.circulo_con, fill=COLOR_VERDE)
            self.label_estado_con.configure(text="Conectado", fg=COLOR_VERDE)
            self.btn_conectar.configure(state=tk.DISABLED)
            self.btn_desconectar.configure(state=tk.NORMAL)
        else:
            self.canvas_estado_con.itemconfigure(self.circulo_con, fill=COLOR_ROJO)
            self.label_estado_con.configure(text="Desconectado", fg=COLOR_ROJO)
            self.btn_conectar.configure(state=tk.NORMAL)
            self.btn_desconectar.configure(state=tk.DISABLED)

    def _leer_socket(self):
        buffer = ""
        while self.conectado:
            try:
                dato = self.sock.recv(512)
                if not dato:
                    break
                buffer += dato.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    linea, buffer = buffer.split("\n", 1)
                    linea = linea.strip()
                    if linea:
                        self.cola.put(linea)
            except Exception:
                break
        self.after(0, self._desconectar)

    def enviar(self, cmd):
        if not self.conectado or not self.sock:
            self._log(f"Sin conexion - comando ignorado: {cmd}", "error")
            return
        try:
            self.sock.sendall((cmd + "\n").encode("utf-8"))
            self._log(f"> {cmd}", "enviado")
        except Exception as ex:
            self._log(f"Error al enviar: {ex}", "error")

    # ── Procesamiento de telemetria ───────────────────────────────────────────

    def _drenar_cola(self):
        try:
            while True:
                linea = self.cola.get_nowait()
                self._procesar_json(linea)
        except queue.Empty:
            pass
        self.after(50, self._drenar_cola)

    def _procesar_json(self, linea):
        try:
            datos = json.loads(linea)
        except json.JSONDecodeError:
            return

        # Estado y modo
        modo   = datos.get("modo", "?")
        estado = datos.get("estado", "?")
        t_est  = datos.get("tEstado", 0)
        hueco  = datos.get("hueco", 0.0)

        # Fix Bug 6: Solo loguear cuando hay un cambio real de estado o modo
        if modo != self.ultimo_modo or estado != self.ultimo_estado:
            self._log(f"Cambio -> Modo: {modo} | Estado: {estado}", "evento")
            self.ultimo_modo = modo
            self.ultimo_estado = estado

        color_modo = {
            "MANUAL":         COLOR_AZUL,
            "TEST_CIEGO":     COLOR_AMARILLO,
            "ESTACIONAR_DER": COLOR_NARANJA,
            "ESTACIONADO":    COLOR_VERDE,
            "ABORTADO":       COLOR_ROJO,
        }.get(modo, COLOR_TEXTO)

        self.label_modo.configure(text=f"Modo:   {modo}", fg=color_modo)
        self.label_estado.configure(text=f"Estado: {estado}")
        self.label_tiempo.configure(text=f"En estado: {t_est} ms")
        self.label_hueco.configure(text=f"Hueco medido: {hueco:.1f} cm")

        # Sensores (Fix Bug 5: Barra llena con 999)
        MAX_CM = 100.0
        for clave, val_key in [("dR", "dR"), ("dL", "dL"), ("dB", "dB"), ("dF", "dF")]:
            val = datos.get(val_key, 999)
            canvas, barra = self.barras_sensor[clave]
            canvas.update_idletasks()
            w = canvas.winfo_width()
            
            if val >= 999:
                proporcion = 0.0
                color = COLOR_TEXTO_DIM # Gris para indicar que no hay lectura
            else:
                proporcion = min(val / MAX_CM, 1.0)
                color = (COLOR_VERDE if val > 20 else COLOR_AMARILLO if val > 10 else COLOR_ROJO)
                
            ancho_barra = int(w * proporcion)
            canvas.itemconfigure(barra, fill=color)
            canvas.coords(barra, 0, 0, ancho_barra, 16)
            txt = f"{val:.0f} cm" if val < 999 else "--- cm"
            self.labels_sensor[clave].configure(text=txt, fg=color)

        # Traccion
        pwm_trac = datos.get("velTrac", 0)
        self.canvas_trac.update_idletasks()
        w_trac = self.canvas_trac.winfo_width()
        prop_trac = min(abs(pwm_trac) / 255.0, 1.0)
        self.canvas_trac.coords(self.barra_trac, 0, 0, int(w_trac * prop_trac), 16)
        self.label_trac.configure(text=f"{pwm_trac} PWM")

        # Posicion direccion
        pos_dir = datos.get("posDir", 0)
        tope    = self.vars_slider.get("tiempoDireccionTope",
                                       tk.IntVar(value=400)).get()
        self.canvas_dir.update_idletasks()
        w_dir = self.canvas_dir.winfo_width()
        prop_dir = min(pos_dir / max(tope, 1), 1.0)
        self.canvas_dir.coords(self.barra_dir, 0, 0, int(w_dir * prop_dir), 16)
        self.label_dir.configure(text=f"{pos_dir} ms")

        # LED freno
        freno = datos.get("freno", False)
        color_freno = self.circulo_freno_color if freno else COLOR_BOTON
        c = getattr(self, "circulo_freno_canvas")
        c.itemconfigure(self.circulo_freno, fill=color_freno)

        # Intermitentes
        inter = datos.get("inter", "OFF")
        c_izq = getattr(self, "circulo_inter_izq_canvas")
        c_der = getattr(self, "circulo_inter_der_canvas")
        on_izq = inter in ("IZQ", "AMBAS", "FIJAS")
        on_der = inter in ("DER", "AMBAS", "FIJAS")
        c_izq.itemconfigure(self.circulo_inter_izq,
                            fill=self.circulo_inter_izq_color if on_izq else COLOR_BOTON)
        c_der.itemconfigure(self.circulo_inter_der,
                            fill=self.circulo_inter_der_color if on_der else COLOR_BOTON)


if __name__ == "__main__":
    app = App()
    app.mainloop()
