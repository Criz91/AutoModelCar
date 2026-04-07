import asyncio
import websockets
import keyboard

ESP32_IP = "192.168.4.1"
WS_URL   = f"ws://{ESP32_IP}/ws"

async def main():
    print(f"Conectando al carrito en {WS_URL} ...")
    try:
        async with websockets.connect(WS_URL) as ws:
            print("Conectado exitosamente")
            print("Controles:")
            print(" W/S = Adelante / Atras")
            print(" A/D = Izquierda / Derecha")
            print(" P   = Iniciar Estacionamiento Autonomo")
            print(" ESC = Freno de emergencia total")

            ultimo_drive = None
            ultimo_steer = None

            while True:
                # Traccion
                if   keyboard.is_pressed("w"): drive = "W"
                elif keyboard.is_pressed("s"): drive = "S"
                else:                          drive = "X"

                # Direccion
                if   keyboard.is_pressed("a"): steer = "A"
                elif keyboard.is_pressed("d"): steer = "D"
                else:                          steer = "C"

                # Enviar solo si cambio la tecla para no saturar el Wi-Fi
                if drive != ultimo_drive:
                    await ws.send(drive)
                    ultimo_drive = drive

                if steer != ultimo_steer:
                    await ws.send(steer)
                    ultimo_steer = steer

                # Activar Autonomo
                if keyboard.is_pressed("p"):
                    await ws.send("P")
                    print("Comando Autonomo Enviado")
                    await asyncio.sleep(0.5)
                    ultimo_drive = None 
                    ultimo_steer = None
                    continue

                # Parada de Emergencia
                if keyboard.is_pressed("esc"):
                    await ws.send("X")
                    await ws.send("C")
                    print("FRENO DE EMERGENCIA")
                    break

                await asyncio.sleep(0.04)

    except Exception as e:
        print("ERROR: Asegurate de estar conectado a la red WiFi ESP32_CAR1")

if __name__ == "__main__":
    asyncio.run(main())