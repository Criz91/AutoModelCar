#PREVIO
Se necesita crear un entorno virtual para instalar las dependencias 
y que no se instalen directamente en tu computador
Ejecuta
    python -m venv venv
    venv\Scripts\activate
    pip install -r requirements.txt

y para ejecutar el codigo usas
    python control_gui.py

#Carpeta sketch_jan26a 
Es una carpeta con el codigo de Arduino programado en C++
En este se esta programando el carrito para recibir ordenes por el puerto serial
Este las lee y en base a ellas realiza los comandos de movimiento.


#contro_car.py 
Version inicial, no tomada en cuenta
#control_gui.py
Version de control del carrito mediante Wifi
De manera previa y como requisito, se debe de conectar a la señal Wi-Fi del ESP32
Con esto el programa se enlaza y se puede controlar el carrito bien con la interfaz grafica
o con las teclas de tu computadora