"""ETAPA 1: comprobar que la placa responde por USB serie (PING -> PONG).

Antes de correrlo:
  1. Sube a la placa el firmware (firmware/PIntegrador_Brazo/PIntegrador_Brazo.ino).
  2. CIERRA el Monitor Serie del IDE de Arduino (si esta abierto, el puerto
     esta ocupado y este script no puede abrirlo).
  3. pip install pyserial   (en el entorno de PyCharm)
"""
import sys
import time

import serial
from serial.tools import list_ports

BAUD = 115200
PINGS = 5


def choose_port():
    ports = list(list_ports.comports())
    # En Mac el CH340 aparece como /dev/cu.usbserial-* o /dev/cu.wchusbserial*
    usb = [p for p in ports if "usb" in p.device.lower()]

    print("Puertos serie encontrados:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  ({p.description})")

    if len(usb) == 1:
        print(f"\nUsando automaticamente: {usb[0].device}")
        return usb[0].device

    if not ports:
        print("\nNo se ve ningun puerto. Revisa el cable (debe ser de datos) y el "
              "controlador CH340 (carpeta 04_Ch340_USB_Driver).")
        sys.exit(1)

    choice = input("\nElige el numero del puerto de la placa: ").strip()
    return ports[int(choice)].device


def main():
    port = choose_port()
    with serial.Serial(port, BAUD, timeout=1) as ser:
        # Al abrir el puerto, la placa puede reiniciarse: esperamos a que arranque
        time.sleep(2.0)
        ser.reset_input_buffer()

        ok = 0
        for n in range(1, PINGS + 1):
            t0 = time.perf_counter()
            ser.write(b"PING\n")
            reply = ser.readline().decode(errors="ignore").strip()
            ms = (time.perf_counter() - t0) * 1000
            good = reply == "PONG"
            ok += good
            print(f"PING {n}: respuesta={reply!r}  tiempo={ms:.1f} ms  {'OK' if good else 'FALLO'}")
            time.sleep(0.3)

        print(f"\nResultado: {ok}/{PINGS} respuestas correctas")
        if ok == PINGS:
            print("La placa responde. Puedes pasar a la Etapa 2.")
        else:
            print("Algo falla: revisa puerto, velocidad (115200) y que el sketch de la Etapa 1 este cargado.")


if __name__ == "__main__":
    main()
