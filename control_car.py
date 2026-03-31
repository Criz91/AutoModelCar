import asyncio
import websockets
import keyboard

ESP32_IP = "192.168.4.1"
WS_URL = f"ws://{ESP32_IP}/ws"

async def main():
    async with websockets.connect(WS_URL) as ws:
        print("Conectado al ESP32")

        while True:
            if keyboard.is_pressed("w"): await ws.send("W")
            elif keyboard.is_pressed("s"): await ws.send("S")
            else: await ws.send("X")

            if keyboard.is_pressed("a"): await ws.send("A")
            elif keyboard.is_pressed("d"): await ws.send("D")
            else: await ws.send("C")

            if keyboard.is_pressed("esc"):
                await ws.send("X")
                await ws.send("C")
                break

            await asyncio.sleep(0.03)

asyncio.run(main())
