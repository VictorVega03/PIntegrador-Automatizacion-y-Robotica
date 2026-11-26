"""ETAPA 2: mandar comandos a la placa desde el teclado.

Comandos: START (obligatorio antes de mover)   A,90   B,120   C,60   D,90   HOME   REL   STATUS   PING   SAFE   RESET
El watchdog del firmware pasa a ERROR si pasan 1.5 s sin mensajes en ACTIVE: escribe PING de vez en cuando.
Escribe  q  para salir (al salir se sueltan los servos).

Recuerda cerrar el Monitor Serie del IDE antes de correrlo.
"""
import time

import serial

from etapa1_puerto_serie import choose_port, BAUD

AYUDA = """
Comandos:
  A,90   mueve el servo A a 90 grados (letras A, B, C, D)
  HOME   regresa los servos activos a su posicion inicial
  REL    suelta los servos (sin fuerza)
  STATUS muestra el estado de cada servo
  PING   prueba de comunicacion
  q      salir
"""


def main():
    port = choose_port()
    with serial.Serial(port, BAUD, timeout=0.2) as ser:
        time.sleep(2.0)            # la placa puede reiniciarse al abrir el puerto
        ser.reset_input_buffer()
        print(AYUDA)

        while True:
            try:
                cmd = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if cmd.lower() in ("q", "salir", "exit"):
                break
            if not cmd:
                continue

            ser.write((cmd + "\n").encode())
            t_end = time.time() + 0.5
            while time.time() < t_end:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print("<", line)

        ser.write(b"REL\n")
        print("Servos liberados. Fin.")


if __name__ == "__main__":
    main()
