"""ETAPA 4: control del brazo con el teclado (WASD / flechas) y calibracion de rangos.

Se usa una ventana de OpenCV: la ventana debe estar ENFOCADA (haz clic en ella)
para recibir las teclas.

Teclas:
  A / D  o  flechas izq / der     base: izquierda / derecha          (servo A)
  W / S  o  flechas arriba / abajo brazo inferior: arriba / abajo    (servo B)
  R / F                            antebrazo: arriba / abajo          (servo C)
  E / Q                            pinza: abrir / cerrar              (servo D)
  1 / 2 / 3                        tamano del paso: 1 / 2 / 5 grados
  [  /  ]                          marcar MINIMO / MAXIMO del servo activo
                                   (el ultimo servo que moviste)
  M                                MONTAJE: los 4 servos a 90 grados
  T                                TRABAJO: A=90, B=120, C=60, D=90
  X                                SOLTAR TODO (emergencia; el brazo puede caer)
  ESC                              salir (los servos se quedan sosteniendo)

Calibracion: mueve un servo hacia un lado con pasos pequenos. En cuanto toque,
zumbe o veas que va a chocar, regresa 2 o 3 grados y marca el limite con [ o ].
Los valores se guardan en config/calibracion.json.

Antes de correrlo: sube el firmware final (firmware/PIntegrador_Brazo) y cierra el Monitor Serie.
El script envia RESET, START y un PING periodico para mantener activo el watchdog del firmware.
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import serial

from etapa1_puerto_serie import choose_port, BAUD

NOMBRES = ["A", "B", "C", "D"]
DESCRIPCION = {"A": "base", "B": "brazo inferior", "C": "antebrazo", "D": "pinza"}
HOME_TRABAJO = {"A": 90, "B": 120, "C": 60, "D": 90}
ANGULO_MIN, ANGULO_MAX = 5, 175      # mismos limites amplios que el firmware
MARGEN = 3                           # grados de seguridad al guardar la calibracion
PASOS = {"1": 1, "2": 2, "3": 5}
ARCHIVO_CALIB = Path(__file__).resolve().parent.parent / "config" / "calibracion.json"
VENTANA = "Control del brazo"

# (servo, signo): signo +1 suma grados, -1 resta.
# Sentidos segun la guia del fabricante: A+ izquierda | B- levantar | C- levantar | D+ abrir
# Si algun movimiento sale al reves, cambia el signo aqui.
TECLAS = {
    "a": ("A", +1), "d": ("A", -1),   # base izquierda / derecha
    "w": ("B", -1), "s": ("B", +1),   # brazo inferior arriba / abajo
    "r": ("C", -1), "f": ("C", +1),   # antebrazo arriba / abajo
    "e": ("D", +1), "q": ("D", -1),   # pinza abrir / cerrar
}
# Codigos de las flechas en OpenCV (Mac, Windows y Linux)
FLECHAS = {
    63234: ("A", +1), 2424832: ("A", +1), 65361: ("A", +1),   # izquierda
    63235: ("A", -1), 2555904: ("A", -1), 65363: ("A", -1),   # derecha
    63232: ("B", -1), 2490368: ("B", -1), 65362: ("B", -1),   # arriba
    63233: ("B", +1), 2621440: ("B", +1), 65364: ("B", +1),   # abajo
}

FONT = cv2.FONT_HERSHEY_SIMPLEX
COLOR_TXT = (225, 225, 225)
COLOR_ACT = (0, 255, 0)
COLOR_ALERTA = (0, 165, 255)


class Brazo:
    def __init__(self, ser):
        self.ser = ser
        self.angulos = {n: 90 for n in NOMBRES}
        self.activo = "A"
        self.paso = 2
        self.limites = {n: {"min": None, "max": None} for n in NOMBRES}
        self.mensajes = []
        self.soltado = False

    def enviar(self, texto):
        self.ser.write((texto + "\n").encode())

    def leer(self):
        while self.ser.in_waiting:
            linea = self.ser.readline().decode(errors="ignore").strip()
            if linea and linea != "PONG" and not linea.startswith("DATA:"):
                self.mensajes.append(linea)
        self.mensajes = self.mensajes[-3:]

    def mover(self, servo, signo):
        if self.soltado:
            return
        self.activo = servo
        nuevo = self.angulos[servo] + signo * self.paso
        nuevo = max(ANGULO_MIN, min(ANGULO_MAX, nuevo))
        if nuevo != self.angulos[servo]:
            self.angulos[servo] = nuevo
            self.enviar(f"{servo},{nuevo}")

    def montaje(self):
        self.enviar("MONTAJE")
        self.angulos = {n: 90 for n in NOMBRES}
        self.soltado = False

    def trabajo(self):
        self.enviar("TRABAJO")
        self.angulos = dict(HOME_TRABAJO)
        self.soltado = False

    def soltar(self):
        self.enviar("REL")
        self.soltado = True

    def marcar(self, tipo):
        if self.soltado:
            return
        self.limites[self.activo][tipo] = self.angulos[self.activo]
        self.guardar()

    def guardar(self):
        datos = {}
        for n in NOMBRES:
            lo, hi = self.limites[n]["min"], self.limites[n]["max"]
            valido = lo is not None and hi is not None and lo + 2 * MARGEN < hi
            datos[n] = {
                "descripcion": DESCRIPCION[n],
                "min_marcado": lo,
                "max_marcado": hi,
                "usar_min": lo + MARGEN if valido else None,
                "usar_max": hi - MARGEN if valido else None,
            }
        datos["_nota"] = (f"usar_min/usar_max = marcado +/- {MARGEN} grados de margen. "
                          "Angulos que se mandan a la placa, no de los topes mecanicos exactos.")
        ARCHIVO_CALIB.write_text(json.dumps(datos, indent=2, ensure_ascii=False), encoding="utf-8")


def dibujar(b):
    img = np.full((560, 800, 3), 30, np.uint8)
    cv2.putText(img, f"Servo activo: {b.activo}    Paso: {b.paso} deg", (20, 35),
                FONT, 0.8, COLOR_ACT, 2)
    if b.soltado:
        cv2.putText(img, "SERVOS SOLTADOS - pulsa M (montaje) o T (trabajo)", (20, 68),
                    FONT, 0.65, COLOR_ALERTA, 2)

    y = 115
    for n in NOMBRES:
        activo = n == b.activo
        color = COLOR_ACT if activo else COLOR_TXT
        ang = b.angulos[n]
        lo, hi = b.limites[n]["min"], b.limites[n]["max"]

        cv2.putText(img, f"{n}  {DESCRIPCION[n]:<14} {ang:3d} deg", (20, y),
                    FONT, 0.65, color, 2 if activo else 1)

        x0, x1 = 380, 770
        cv2.rectangle(img, (x0, y - 16), (x1, y + 4), (90, 90, 90), 1)
        xa = x0 + int((x1 - x0) * ang / 180)
        cv2.rectangle(img, (x0, y - 16), (xa, y + 4), color, -1)
        for val in (lo, hi):
            if val is not None:
                xv = x0 + int((x1 - x0) * val / 180)
                cv2.line(img, (xv, y - 22), (xv, y + 10), (0, 0, 255), 2)

        cv2.putText(img, f"min: {lo if lo is not None else '--'}   max: {hi if hi is not None else '--'}",
                    (380, y + 28), FONT, 0.5, color, 1)
        y += 72

    ayuda = [
        "A/D o flechas izq/der: base | W/S o flechas arriba/abajo: brazo inferior",
        "R/F: antebrazo | E/Q: pinza abrir/cerrar | 1/2/3: paso 1/2/5 deg",
        "[ y ]: marcar MIN / MAX del servo activo | M: montaje | T: trabajo",
        "X: SOLTAR TODO (emergencia) | ESC: salir",
    ]
    y = 410
    for linea in ayuda:
        cv2.putText(img, linea, (20, y), FONT, 0.5, COLOR_TXT, 1)
        y += 24

    y = 520
    for msg in b.mensajes[-2:]:
        cv2.putText(img, "< " + msg[:90], (20, y), FONT, 0.45, (160, 200, 255), 1)
        y += 20
    return img


def main():
    port = choose_port()
    with serial.Serial(port, BAUD, timeout=0.05) as ser:
        time.sleep(2.5)                 # la placa se reinicia al abrir el puerto
        ser.reset_input_buffer()

        b = Brazo(ser)
        b.montaje()                     # todos a 90 grados y sostenidos
        b.enviar("START")               # el firmware solo mueve servos en ACTIVE
        ultimo_ping = time.monotonic()
        cv2.namedWindow(VENTANA)
        print("Ventana abierta: haz clic en ella para que reciba las teclas. ESC para salir.")

        while True:
            b.leer()
            if time.monotonic() - ultimo_ping > 0.2:
                b.enviar("PING")        # latido para el watchdog
                ultimo_ping = time.monotonic()
            cv2.imshow(VENTANA, dibujar(b))
            if cv2.getWindowProperty(VENTANA, cv2.WND_PROP_VISIBLE) < 1:
                break

            k = cv2.waitKeyEx(15)
            if k == -1:
                continue
            if k in FLECHAS:
                b.mover(*FLECHAS[k])
                continue
            if k == 27:                 # ESC
                break
            if 0 <= k < 256:
                c = chr(k).lower()
                if c in TECLAS:
                    b.mover(*TECLAS[c])
                elif c in PASOS:
                    b.paso = PASOS[c]
                elif c == "[":
                    b.marcar("min")
                elif c == "]":
                    b.marcar("max")
                elif c == "m":
                    b.montaje()
                elif c == "t":
                    b.trabajo()
                elif c == "x":
                    b.soltar()

        b.guardar()
        cv2.destroyAllWindows()

    print(f"\nCalibracion guardada en {ARCHIVO_CALIB}")
    print("Los servos quedaron sosteniendo su posicion. Baja el brazo con la mano antes de apagar el interruptor.")
    for n in NOMBRES:
        print(f"  {n} ({DESCRIPCION[n]}): min={b.limites[n]['min']}  max={b.limites[n]['max']}")


if __name__ == "__main__":
    main()
