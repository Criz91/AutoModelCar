import asyncio
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import websockets

ESP32_IP = "192.168.4.1"
WS_URL = f"ws://{ESP32_IP}/ws"


class CarControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Car Controller (WebSocket)")
        self.root.geometry("560x520")

        # Variables de estado y conexion
        self.ws = None
        self.connected = False

        # Almacena los ultimos comandos enviados
        self.last_drive = "X"  # W/S/X
        self.last_steer = "C"  # A/D/C

        # Variables para las velocidades visuales en la interfaz
        self.drive_speed = tk.IntVar(value=180)
        self.steer_speed = tk.IntVar(value=160)

        self._build_ui()

        # Configura y arranca el loop de asyncio en un hilo separado
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        # Vinculacion de eventos de teclado
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.pressed = set()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=12)
        frm.pack(fill="both", expand=True)

        # Seccion de conexion
        conn_frame = ttk.LabelFrame(frm, text="Conexion", padding=10)
        conn_frame.pack(fill="x")

        self.lbl_status = ttk.Label(conn_frame, text="Desconectado")
        self.lbl_status.pack(side="left")

        ttk.Button(conn_frame, text="Conectar", command=self.connect).pack(side="right", padx=6)
        ttk.Button(conn_frame, text="Desconectar", command=self.disconnect).pack(side="right", padx=6)

        # Seccion de estado actual
        state_frame = ttk.LabelFrame(frm, text="Estado (lo que se manda al ESP32)", padding=10)
        state_frame.pack(fill="x", pady=10)

        self.lbl_drive = ttk.Label(state_frame, text="Traccion: X (STOP)")
        self.lbl_drive.pack(anchor="w")

        self.lbl_steer = ttk.Label(state_frame, text="Direccion: C (CENTER)")
        self.lbl_steer.pack(anchor="w")

        # Seccion de controles manuales
        ctrl_frame = ttk.LabelFrame(frm, text="Controles manuales (WASD)", padding=10)
        ctrl_frame.pack(fill="x")

        row1 = ttk.Frame(ctrl_frame)
        row1.pack(fill="x", pady=3)
        ttk.Button(row1, text="W (Forward)", command=lambda: self.set_drive("W")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row1, text="X (Stop)",      command=lambda: self.set_drive("X")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row1, text="S (Reverse)", command=lambda: self.set_drive("S")).pack(side="left", expand=True, fill="x", padx=3)

        row2 = ttk.Frame(ctrl_frame)
        row2.pack(fill="x", pady=3)
        ttk.Button(row2, text="A (Left)",   command=lambda: self.set_steer("A")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row2, text="C (Center)",   command=lambda: self.set_steer("C")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row2, text="D (Right)",  command=lambda: self.set_steer("D")).pack(side="left", expand=True, fill="x", padx=3)

        ttk.Button(ctrl_frame, text="FULL STOP (X + C)", command=self.full_stop).pack(fill="x", pady=6)

        # Seccion de modo autonomo
        auto_frame = ttk.LabelFrame(frm, text="Modo autonomo", padding=10)
        auto_frame.pack(fill="x", pady=10)

        park_row = ttk.Frame(auto_frame)
        park_row.pack(fill="x", pady=3)
        ttk.Button(park_row, text="Estacionar IZQUIERDA",
                   command=lambda: self.start_park("PL")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(park_row, text="Estacionar DERECHA",
                   command=lambda: self.start_park("PR")).pack(side="left", expand=True, fill="x", padx=3)

        ttk.Button(auto_frame, text="Toggle Sensor Test (T)",
                   command=lambda: self.send("T")).pack(fill="x", pady=(2, 0))
        ttk.Button(auto_frame, text="Toggle Intermitentes (H)",
                   command=lambda: self.send("H")).pack(fill="x", pady=(2, 0))

        info = ttk.Label(frm, text="Tip: WASD con esta ventana enfocada. Esc o cualquier tecla = cancelar AUTO.")
        info.pack(anchor="w", pady=(8, 0))

    def _run_loop(self):
        # Establece y corre el ciclo de eventos en este hilo
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    # Metodos de conexion WebSocket
    def connect(self):
        asyncio.run_coroutine_threadsafe(self._connect_ws(), self.loop)

    async def _connect_ws(self):
        try:
            if self.ws:
                await self.ws.close()
            self.ws = await websockets.connect(WS_URL)
            self.connected = True
            self._ui_status(True)
        except Exception as e:
            self.connected = False
            self.ws = None
            self._ui_status(False)
            self.root.after(0, lambda: messagebox.showerror("Error", f"No se pudo conectar:\n{e}"))

    def disconnect(self):
        asyncio.run_coroutine_threadsafe(self._disconnect_ws(), self.loop)

    async def _disconnect_ws(self):
        try:
            if self.ws:
                await self.ws.close()
        finally:
            self.ws = None
            self.connected = False
            self._ui_status(False)

    def _ui_status(self, ok: bool):
        # Actualiza el texto de estado en el hilo de la interfaz
        def _do():
            self.lbl_status.config(text="Conectado" if ok else "Desconectado")
        self.root.after(0, _do)

    # Metodos para enviar datos
    def send(self, msg: str):
        asyncio.run_coroutine_threadsafe(self._send(msg), self.loop)

    async def _send(self, msg: str):
        if not self.ws:
            return
        try:
            await self.ws.send(msg)
        except Exception:
            # En caso de error, resetea la conexion
            self.ws = None
            self.connected = False
            self._ui_status(False)

    # Logica de actualizacion de comandos
    def set_drive(self, cmd):
        self.last_drive = cmd
        if cmd == "W":
            self.lbl_drive.config(text="Traccion: W (FORWARD)")
        elif cmd == "S":
            self.lbl_drive.config(text="Traccion: S (REVERSE)")
        else:
            self.lbl_drive.config(text="Traccion: X (STOP)")
        self.send(cmd)

    def set_steer(self, cmd):
        self.last_steer = cmd
        if cmd == "A":
            self.lbl_steer.config(text="Direccion: A (LEFT)")
        elif cmd == "D":
            self.lbl_steer.config(text="Direccion: D (RIGHT)")
        else:
            self.lbl_steer.config(text="Direccion: C (CENTER)")
        self.send(cmd)

    def full_stop(self):
        self.set_drive("X")
        self.set_steer("C")

    def start_park(self, cmd):
        side = "DERECHA" if cmd == "PR" else "IZQUIERDA"
        self.lbl_drive.config(text=f"Traccion: AUTO PARK {side}")
        self.lbl_steer.config(text=f"Direccion: AUTO PARK {side}")
        self.send(cmd)

    # Manejo de eventos de teclado
    def on_key_press(self, event):
        k = event.keysym.lower()
        self.pressed.add(k)

        if k == "w":
            self.set_drive("W")
        elif k == "s":
            self.set_drive("S")
        elif k == "a":
            self.set_steer("A")
        elif k == "d":
            self.set_steer("D")
        elif k == "escape":
            self.full_stop()

    def on_key_release(self, event):
        k = event.keysym.lower()
        if k in self.pressed:
            self.pressed.remove(k)

        if k in ("w", "s"):
            self.set_drive("X")
        if k in ("a", "d"):
            self.set_steer("C")

    def on_close(self):
        # Detiene el vehiculo y cierra la conexion antes de salir
        try:
            self.full_stop()
        except Exception:
            pass
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = CarControllerGUI(root)
    root.mainloop()