# CarroTMR1 — Sistema de Estacionamiento Autónomo
# Torneo Mexicano de Robótica 2026

## Setup

Crea el entorno virtual e instala dependencias:

    python -m venv venv
    venv\Scripts\activate        (Windows)
    source venv/bin/activate     (Mac/Linux)
    pip install -r requirements.txt

## Cómo usar

1. Conecta tu computadora a la red WiFi del carro: **ESP32_CAR1** (contraseña: 12345678)
2. Ejecuta el control:

    python control_car.py       ← teclado simple (recomendado)
    python control_gui.py       ← interfaz gráfica con botones

## Teclas (control_car.py)

| Tecla | Acción |
|-------|--------|
| W     | Avanzar |
| S     | Retroceder |
| A     | Girar izquierda |
| D     | Girar derecha |
| **P** | **Iniciar estacionamiento autónomo** |
| ESC   | Parada de emergencia + salir |

## Override de seguridad

Durante el modo autónomo, presionar **cualquier tecla W/S/A/D**
regresa inmediatamente al control manual y frena el carro.

## Modos de operación

**MANUAL** (default): el operador controla el carro con WASD.
Obligatorio para actuar como obstáculo en otras pruebas del TMR.

**AUTÓNOMO** (tecla P): el carro busca el hueco, se centra,
enciende intermitentes y se estaciona solo. Las intermitentes
parpadean continuamente hasta que el carro queda estacionado.

## Cambios de pines importantes (vs versión anterior)

Si actualizas el firmware, nota estos cambios para evitar conflictos:

| Señal         | Pin anterior | Pin nuevo | Motivo |
|---------------|-------------|-----------|--------|
| LED_IZQUIERDO | 11          | **2**     | 11 = IN4 (motor tracción) |
| ECHO_ESQUINA  | 18          | **21**    | 18 = ENA (PWM dirección) |

## Estructura del proyecto

    CarroTMR1/
    ├── sketch_jan26a/
    │   └── sketch_jan26a.ino   ← Firmware ESP32 (motores + sensores + WebSocket)
    ├── control_car.py           ← Control por teclado (versión activa)
    ├── control_gui.py           ← Control por GUI (versión alternativa)
    ├── requirements.txt
    └── README.txt

## requirements.txt

    websockets==16.0
    keyboard
