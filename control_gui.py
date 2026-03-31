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
        self.root.geometry("520x360")

        # Estado / conexión
        self.ws = None
        self.connected = False

        # Últimos comandos enviados
        self.last_drive = "X"  # W/S/X
        self.last_steer = "C"  # A/D/C

        # Velocidades (para mandar al ESP32 como mensaje opcional)
        self.drive_speed = tk.IntVar(value=180)
        self.steer_speed = tk.IntVar(value=160)

        # Construir UI
        self._build_ui()

        # Loop asyncio en hilo aparte
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        # Bindeo de teclas (solo funciona cuando esta ventana está enfocada)
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.pressed = set()

        # Cierre limpio
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=12)
        frm.pack(fill="both", expand=True)

        # Conexión
        conn_frame = ttk.LabelFrame(frm, text="Conexión", padding=10)
        conn_frame.pack(fill="x")

        self.lbl_status = ttk.Label(conn_frame, text="Desconectado")
        self.lbl_status.pack(side="left")

        ttk.Button(conn_frame, text="Conectar", command=self.connect).pack(side="right", padx=6)
        ttk.Button(conn_frame, text="Desconectar", command=self.disconnect).pack(side="right", padx=6)

        # Estado comandos
        state_frame = ttk.LabelFrame(frm, text="Estado (lo que se manda al ESP32)", padding=10)
        state_frame.pack(fill="x", pady=10)

        self.lbl_drive = ttk.Label(state_frame, text="Tracción: X (STOP)")
        self.lbl_drive.pack(anchor="w")

        self.lbl_steer = ttk.Label(state_frame, text="Dirección: C (CENTER)")
        self.lbl_steer.pack(anchor="w")

        # Controles (botones)
        ctrl_frame = ttk.LabelFrame(frm, text="Controles", padding=10)
        ctrl_frame.pack(fill="x")

        row1 = ttk.Frame(ctrl_frame)
        row1.pack(fill="x", pady=3)
        ttk.Button(row1, text="↑ W (Forward)", command=lambda: self.set_drive("W")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row1, text="X (Stop)", command=lambda: self.set_drive("X")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row1, text="↓ S (Reverse)", command=lambda: self.set_drive("S")).pack(side="left", expand=True, fill="x", padx=3)

        row2 = ttk.Frame(ctrl_frame)
        row2.pack(fill="x", pady=3)
        ttk.Button(row2, text="← A (Left)", command=lambda: self.set_steer("A")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row2, text="C (Center)", command=lambda: self.set_steer("C")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row2, text="→ D (Right)", command=lambda: self.set_steer("D")).pack(side="left", expand=True, fill="x", padx=3)

        ttk.Button(ctrl_frame, text="FULL STOP (X + C)", command=self.full_stop).pack(fill="x", pady=6)

        # Sliders (opcional: mando velocidades al ESP32 si tú implementas ese comando)
        spd_frame = ttk.LabelFrame(frm, text="Velocidades (opcional)", padding=10)
        spd_frame.pack(fill="x", pady=10)

        ttk.Label(spd_frame, text="Tracción (driveSpeed)").pack(anchor="w")
        ttk.Scale(spd_frame, from_=0, to=255, orient="horizontal",
                  variable=self.drive_speed, command=lambda e: self.update_speed_labels()).pack(fill="x")

        ttk.Label(spd_frame, text="Dirección (steerSpeed)").pack(anchor="w", pady=(8, 0))
        ttk.Scale(spd_frame, from_=0, to=255, orient="horizontal",
                  variable=self.steer_speed, command=lambda e: self.update_speed_labels()).pack(fill="x")

        self.lbl_speeds = ttk.Label(spd_frame, text="driveSpeed=180 | steerSpeed=160")
        self.lbl_speeds.pack(anchor="w", pady=(6, 0))

        info = ttk.Label(frm, text="Tip: usa WASD con esta ventana enfocada. Esc = FULL STOP.")
        info.pack(anchor="w")

    def update_speed_labels(self):
        self.lbl_speeds.config(text=f"driveSpeed={self.drive_speed.get()} | steerSpeed={self.steer_speed.get()}")

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
        def _do():
            self.lbl_status.config(text="Conectado ✅" if ok else "Desconectado ❌")
        self.root.after(0, _do)

    def send(self, msg: str):
        asyncio.run_coroutine_threadsafe(self._send(msg), self.loop)

    async def _send(self, msg: str):
        if not self.ws:
            return
        try:
            await self.ws.send(msg)
        except Exception:
            # si algo truena, marca desconectado
            self.ws = None
            self.connected = False
            self._ui_status(False)

    def set_drive(self, cmd):
        self.last_drive = cmd
        if cmd == "W":
            self.lbl_drive.config(text="Tracción: W (FORWARD)")
        elif cmd == "S":
            self.lbl_drive.config(text="Tracción: S (REVERSE)")
        else:
            self.lbl_drive.config(text="Tracción: X (STOP)")

        # manda al ESP32
        self.send(cmd)

    def set_steer(self, cmd):
        self.last_steer = cmd
        if cmd == "A":
            #self.lbl_steer.config(text="Dirección: D (RIGHT)")
            self.lbl_steer.config(text="Dirección: A (LEFT)")
        elif cmd == "D":
            #self.lbl_steer.config(text="Dirección: A (LEFT)")
            self.lbl_steer.config(text="Dirección: D (RIGHT)")
        else:
            self.lbl_steer.config(text="Dirección: C (CENTER)")

        # manda al ESP32
        self.send(cmd)

    def full_stop(self):
        self.set_drive("X")
        self.set_steer("C")

    # --- teclas ---
    def on_key_press(self, event):
        k = event.keysym.lower()
        self.pressed.add(k)

        if k == "w":
            self.set_drive("W")
        elif k == "s":
            self.set_drive("S")
        elif k == "a":
            self.set_steer("D")
        elif k == "d":
            self.set_steer("A")
        elif k == "escape":
            self.full_stop()

    def on_key_release(self, event):
        k = event.keysym.lower()
        if k in self.pressed:
            self.pressed.remove(k)

        # al soltar W/S -> stop
        if k in ("w", "s"):
            self.set_drive("X")

        # al soltar A/D -> center
        if k in ("a", "d"):
            self.set_steer("C")

    def on_close(self):
        # al cerrar, manda stop
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
