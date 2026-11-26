# Control gestual de un brazo robótico (MediaPipe + ESP32-C3)

Proyecto Integrador de Automatización y Robótica (UAT, Facultad de Ingeniería).
Un brazo eArm (SIYEENOVE E1R0000, 4 servos MG90S) se controla en tiempo real con los gestos de la mano.
Python en la computadora detecta la mano con MediaPipe y manda comandos de texto al ESP32-C3 por Wi-Fi (TCP) o por USB;
el ESP32 los valida con una máquina de estados y mueve los servos.

```
Cámara -> MediaPipe -> anti-ruido -> decisión (Python) --texto por Wi-Fi/USB--> FSM + watchdog (ESP32) -> servos
```

## Estructura

```
src/                       Código de Python
  main.py                    Programa principal: gestos, filtros y envío de comandos
  enlace.py                  Enlace con el brazo: socket TCP (Wi-Fi) o puerto serie (USB)
  etapa1_puerto_serie.py     Selección del puerto USB y velocidad (115200)
  etapa2_comandos.py         Consola para mandar comandos a mano (pruebas)
  etapa4_teclado.py          Control con teclado y calibración de los límites de cada servo
config/calibracion.json    Límites calibrados de los servos
firmware/PIntegrador_Brazo/  Firmware del ESP32-C3 (Arduino)
models/                    Modelo hand_landmarker.task (se descarga solo)
tests/                     Pruebas de Python y del firmware (sin necesidad del brazo)
docs/                      Notas de trabajo
requirements.txt           Librerías de Python
```

## Requisitos

- Python 3.9 a 3.12 (probado con 3.11). Instalación: `pip install -r requirements.txt`
- Arduino IDE con el core de ESP32 y la librería ESP32Servo. Placa: "ESP32C3 Dev Module".
- Brazo eArm con batería 18650 cargada (el riel de los servos solo se alimenta de la batería).

## Uso

1. Abre `firmware/PIntegrador_Brazo/PIntegrador_Brazo.ino`, súbelo a la placa y **cierra el Monitor Serie**.
2. Por Wi-Fi: conecta la computadora a la red `eArm_Gestos` (clave `brazo1234`). Por USB: conecta el cable.
3. Ejecuta `python src/main.py`, elige la cámara y el enlace (1 Wi-Fi, 2 USB).
4. Pon la mano frente a la cámara y pulsa **C** para activar el control.

### Gestos (modo relativo, como un ratón)

| Gesto | Acción |
|---|---|
| Mano abierta o puño | Izquierda/derecha mueve la base; arriba/abajo el brazo; la apertura de los dedos controla la pinza |
| Girar la mano (como una perilla) | Mueve el antebrazo |
| Paz u otro gesto, o sin mano | Embrague: el brazo se queda quieto |
| Pulgar arriba / abajo | Sube / baja el antebrazo |
| OK | Modo fino |
| Cuernos | Pausa / reanudar |
| Llamada | Posición de trabajo |

### Teclas

C activar · ESPACIO pausa · F modo fino · U/J antebrazo · M posición de montaje · T posición de trabajo ·
**X paro de emergencia** · L soltar servos · ESC salir.

## Seguridad

El firmware tiene cuatro estados: IDLE, ACTIVE, SAFE y ERROR. Solo ACTIVE mueve los servos.
Si pasan 1.5 s sin mensajes en ACTIVE (watchdog), pasa a ERROR: el brazo baja solo a una pose de reposo y suena el zumbador.
Python envía un `PING` cada 0.1 s desde un hilo propio para mantener vivo el enlace.

## Calibración

`python src/etapa4_teclado.py` abre una ventana para mover cada servo con el teclado y marcar su mínimo y máximo
(teclas `[` y `]`). Los valores se guardan en `config/calibracion.json`.

## Pruebas

```
python tests/test_gestos.py
g++ -std=c++17 -Itests/firmware/stubs -x c++ tests/firmware/test_fw.cpp -o /tmp/test_fw && /tmp/test_fw
```

La primera prueba verifica los gestos, los filtros y el enlace TCP con un servidor simulado. La segunda compila
el firmware en la computadora y verifica la máquina de estados y el watchdog.

## Limitaciones conocidas

- La batería de 1500 mAh entrega poca corriente: con los cuatro servos en movimiento puede haber un bajón de voltaje
  (brownout) que reinicia el ESP32 y corta el enlace Wi-Fi. Por USB el enlace es estable.
  El firmware imprime `INFO:reset=<n>` al conectarse (9 = brownout).
- Una sola mano; el paro de emergencia es la tecla X del teclado (no hay botón físico).
