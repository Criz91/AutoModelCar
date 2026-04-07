import asyncio
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import websockets

# Direccion IP por defecto del ESP32
ESP32_IP = "192.168.4.1"
WS_URL = f"ws://{ESP32_IP}/ws"

class CarControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Control TMR - Modo Hibrido")
        self.root.geometry("550x420") 

        # Estado de conexion
        self.ws = None
        self.connected = False

        # Ultimos comandos enviados
        self.last_drive = "X" 
        self.last_steer = "C" 

        self._build_ui()

        # Iniciar el Loop de asyncio en un hilo aparte
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        # Controles por teclado
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.pressed = set()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=12)
        frm.pack(fill="both", expand=True)

        # Seccion Conexion
        conn_frame = ttk.LabelFrame(frm, text="Conexion Wi-Fi", padding=10)
        conn_frame.pack(fill="x")

        self.lbl_status = ttk.Label(conn_frame, text="Desconectado")
        self.lbl_status.pack(side="left")

        ttk.Button(conn_frame, text="Conectar", command=self.connect).pack(side="right", padx=6)
        ttk.Button(conn_frame, text="Desconectar", command=self.disconnect).pack(side="right", padx=6)

        # Seccion Estado
        state_frame = ttk.LabelFrame(frm, text="Estado Actual", padding=10)
        state_frame.pack(fill="x", pady=10)

        self.lbl_drive = ttk.Label(state_frame, text="Traccion: X (FRENADO)")
        self.lbl_drive.pack(anchor="w")

        self.lbl_steer = ttk.Label(state_frame, text="Direccion: C (CENTRO)")
        self.lbl_steer.pack(anchor="w")

        # Seccion Controles Manuales
        ctrl_frame = ttk.LabelFrame(frm, text="Controles Manuales", padding=10)
        ctrl_frame.pack(fill="x")

        row1 = ttk.Frame(ctrl_frame)
        row1.pack(fill="x", pady=3)
        ttk.Button(row1, text="W (Adelante)", command=lambda: self.set_drive("W")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row1, text="X (Frenar)", command=lambda: self.set_drive("X")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row1, text="S (Reversa)", command=lambda: self.set_drive("S")).pack(side="left", expand=True, fill="x", padx=3)

        row2 = ttk.Frame(ctrl_frame)
        row2.pack(fill="x", pady=3)
        ttk.Button(row2, text="A (Izquierda)", command=lambda: self.set_steer("A")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row2, text="C (Centro)", command=lambda: self.set_steer("C")).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row2, text="D (Derecha)", command=lambda: self.set_steer("D")).pack(side="left", expand=True, fill="x", padx=3)

        # Seccion TMR Autonomo
        auto_frame = ttk.LabelFrame(frm, text="Pruebas Torneo Mexicano de Robotica", padding=10)
        auto_frame.pack(fill="x", pady=10)

        ttk.Button(auto_frame, text="INICIAR ESTACIONAMIENTO AUTONOMO (P)", command=self.start_autonomous).pack(fill="x", pady=2)
        ttk.Button(auto_frame, text="FRENO DE EMERGENCIA (Esc)", command=self.full_stop).pack(fill="x", pady=6)

        info = ttk.Label(frm, text="Tip: Usa WASD en tu teclado. Presiona P para estacionar. Esc para frenar.")
        info.pack(anchor="w")

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
            self.root.after(0, lambda: messagebox.showerror("Error", f"No se pudo conectar.\nAsegurate de estar en la red del ESP32.\n{e}"))

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
            self.lbl_status.config(text="Conectado" if ok else "Desconectado")
        self.root.after(0, _do)

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
            self._ui_status(False)

    def set_drive(self, cmd):
        self.last_drive = cmd
        if cmd == "W":
            self.lbl_drive.config(text="Traccion: W (ADELANTE)")
        elif cmd == "S":
            self.lbl_drive.config(text="Traccion: S (REVERSA)")
        else:
            self.lbl_drive.config(text="Traccion: X (FRENADO)")
        self.send(cmd)

    def set_steer(self, cmd):
        self.last_steer = cmd
        if cmd == "A":
            self.lbl_steer.config(text="Direccion: A (IZQUIERDA)")
        elif cmd == "D":
            self.lbl_steer.config(text="Direccion: D (DERECHA)")
        else:
            self.lbl_steer.config(text="Direccion: C (CENTRO)")
        self.send(cmd)

    def full_stop(self):
        self.set_drive("X")
        self.set_steer("C")
        self.lbl_drive.config(text="Traccion: X (EMERGENCIA)")

    def start_autonomous(self):
        self.send("P")
        self.lbl_drive.config(text="Traccion: AUTO (Estacionando...)")
        self.lbl_steer.config(text="Direccion: AUTO")

    def on_key_press(self, event):
        k = event.keysym.lower()
        if k in self.pressed: return 
        self.pressed.add(k)

        if k == "w":
            self.set_drive("W")
        elif k == "s":
            self.set_drive("S")
        elif k == "a":
            self.set_steer("D") 
        elif k == "d":
            self.set_steer("A")
        elif k == "p":
            self.start_autonomous()
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