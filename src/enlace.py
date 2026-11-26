"""Enlace con el brazo: Wi-Fi (socket TCP) o USB serie, con la misma interfaz.

Wi-Fi: el ESP32 crea la red "eArm_Gestos" (clave "brazo1234"). Conecta la Mac a esa red
y el brazo queda en 192.168.4.1, puerto 8080. Mensajes de texto terminados en salto de linea.
"""
import select
import socket
import threading
import time

import serial

from etapa1_puerto_serie import choose_port, BAUD

WIFI_SSID = "eArm_Gestos"
WIFI_CLAVE = "brazo1234"
HOST = "192.168.4.1"
PUERTO_TCP = 8080


class EnlaceTCP:
    """Imita lo que usamos de serial.Serial: write, in_waiting, readline, reset_input_buffer, close."""

    def __init__(self, host=HOST, puerto=PUERTO_TCP, espera=4.0):
        self.host, self.puerto = host, puerto
        self.sock = None
        self.motivo = ""
        self._lock = threading.RLock()          # el latido escribe desde otro hilo
        self._conectar(espera)

    def _conectar(self, espera):
        self.sock = socket.create_connection((self.host, self.puerto), timeout=espera)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(1.0)
        self.buf = b""
        self.lineas = []
        self.cerrado = False
        self.motivo = ""

    def _perder(self, motivo):
        if not self.cerrado:
            self.cerrado = True
            self.motivo = motivo

    def reconectar(self):
        """Reintenta la conexion (corto). True si volvio el enlace."""
        with self._lock:
            try:
                if self.sock is not None:
                    self.sock.close()
            except OSError:
                pass
            try:
                self._conectar(1.0)
                return True
            except OSError as e:
                self.cerrado = True
                self.motivo = f"reintento fallido: {e}"
                return False

    def _recibir(self):
        if self.cerrado:
            return
        try:
            while select.select([self.sock], [], [], 0)[0]:
                datos = self.sock.recv(4096)
                if not datos:
                    self._perder("el ESP32 cerro la conexion")
                    break
                self.buf += datos
        except OSError as e:
            self._perder(f"{type(e).__name__}: {e}")
        while b"\n" in self.buf:
            linea, self.buf = self.buf.split(b"\n", 1)
            if linea.startswith(b"INFO:"):      # diagnostico de arranque: se muestra siempre, aunque se limpie el buffer
                print(f"ESP32 {linea.decode(errors='ignore').strip()}  (reset 9 = brownout, 1 = encendido)")
                continue
            self.lineas.append(linea + b"\n")

    @property
    def in_waiting(self):
        self._recibir()
        return len(self.lineas)

    def readline(self):
        self._recibir()
        return self.lineas.pop(0) if self.lineas else b""

    def write(self, datos):
        with self._lock:
            if self.cerrado:
                return 0
            try:
                self.sock.sendall(datos)
                return len(datos)
            except OSError as e:
                self._perder(f"al enviar, {type(e).__name__}: {e}")
                return 0

    def reset_input_buffer(self):
        self._recibir()
        self.buf, self.lineas = b"", []

    def close(self):
        self.cerrado = True
        try:
            self.sock.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def abrir_enlace():
    """Pregunta el medio y devuelve el enlace listo (Wi-Fi o USB)."""
    print("\n=== ENLACE CON EL BRAZO ===")
    print(f"1: Wi-Fi (conecta la Mac a la red '{WIFI_SSID}', clave '{WIFI_CLAVE}')")
    print("2: Cable USB")
    opcion = input("Elige una opcion (1 o 2) [Default: 1]: ").strip()
    if opcion == "2":
        port = choose_port()
        ser = serial.Serial(port, BAUD, timeout=0.02)
        time.sleep(2.5)                              # la placa se reinicia al abrir el puerto
        ser.reset_input_buffer()
        return ser
    try:
        enlace = EnlaceTCP()
    except OSError as e:
        raise SystemExit(
            f"No pude conectar a {HOST}:{PUERTO_TCP} ({e}).\n"
            f"Revisa que la Mac este conectada a la red Wi-Fi '{WIFI_SSID}' y que el brazo este encendido.")
    time.sleep(0.3)
    enlace.reset_input_buffer()
    return enlace
