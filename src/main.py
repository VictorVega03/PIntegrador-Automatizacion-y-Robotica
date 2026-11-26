"""Control del brazo eArm con la mano en tiempo real (MediaPipe + ESP32-C3).

Modo relativo: la mano empuja al brazo como un raton; con el embrague se reposiciona sin moverlo.

Gestos (una mano, camara en espejo):
  Mano abierta / puno ... izquierda-derecha mueve la base (A), arriba-abajo el brazo (B);
                          la apertura de los dedos controla la pinza (D); girar la mano
                          como una perilla mueve el antebrazo (C).
  Paz u otro gesto ...... embrague (el brazo se queda quieto). Sin mano tambien.
  Pulgar arriba / abajo . sube / baja el antebrazo (C) mientras se mantenga.
  OK .................... modo fino   | Cuernos ... pausa / reanudar | Llamada ... posicion de trabajo

Teclas: C activar, ESPACIO pausa, F fino, U/J antebrazo, M montaje, T trabajo,
        X paro de emergencia (SAFE), L soltar servos, ESC salir.
Cierra el Monitor Serie antes de ejecutar. Limites de servos: config/calibracion.json.
"""
import json
import math
import threading
import time
import urllib.request
from pathlib import Path

import cv2

from enlace import abrir_enlace

RAIZ = Path(__file__).resolve().parent.parent          # raiz del proyecto
MODEL_PATH = RAIZ / "models" / "hand_landmarker.task"
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/1/hand_landmarker.task")
ARCHIVO_CALIB = RAIZ / "config" / "calibracion.json"

NOMBRES = ["A", "B", "C", "D"]
DESCRIPCION = {"A": "base", "B": "brazo inferior", "C": "antebrazo", "D": "pinza"}

NEUTRO = {"A": 90, "B": 120, "C": 60, "D": 90}      # posicion de trabajo
LIMITES_DEFECTO = {n: (5, 175) for n in "ABCD"}
USAR_CALIBRACION = True    # True: usar los limites de calibracion.json (etapa4) si existen

# --- Movimiento relativo (A y B) ---
GANANCIA_A = 160.0     # grados de la base por cada "ancho de imagen" que muevas la mano
GANANCIA_B = 160.0     # grados del brazo inferior por cada "alto de imagen"
GANANCIA_C = 1.6       # grados del antebrazo por cada grado que giras la mano
BANDA_ANG = 2.0        # giro minimo (grados de la mano) para que cuente; evita temblor
FACTOR_FINO = 0.35     # multiplicador de ganancia con la tecla F
BANDA = 0.003          # la mano debe moverse al menos esto (fraccion de imagen) para sumar; mata el temblor
JOG_C = 1.5            # grados por cuadro del antebrazo mientras mantienes SENALAR / TRES
MAX_SALTO = 0.12       # un salto mayor que esto en un cuadro (fraccion de imagen) se considera error y se ignora

# --- Gestos de dedos (cociente punta/base medido desde la muneca, en 3D) ---
EXT_ON = 1.45          # por encima: dedo extendido
EXT_OFF = 1.25         # por debajo: dedo doblado (entre ambos se mantiene el estado anterior)
Q_CERRADO = 0.95       # promedio de 4 dedos con el puno cerrado
Q_ABIERTO = 1.75       # idem con la mano abierta
T_CERRADO = 0.55       # pulgar punta -> base del menique / palma, con el pulgar doblado
T_ABIERTO = 1.25       # idem con el pulgar abierto
PINZA_T_CERRADA = 0.20 # apertura total (0..1) a partir de la cual la pinza esta cerrada
PINZA_T_ABIERTA = 0.80 # apertura total a partir de la cual la pinza esta abierta
ACCION_FRAMES = 15      # cuadros seguidos (~0.5 s) para disparar OK / cuernos / llamada
FRAMES_CONFIRMAR = {"abierta": 2, "puno": 2, "paz": 4, "pulgar_arriba": 4, "pulgar_abajo": 4,
                    "ok": ACCION_FRAMES, "cuernos": ACCION_FRAMES, "llamada": ACCION_FRAMES, "otro": 4}
ACCIONES = ("ok", "cuernos", "llamada")
OK_DIST = 0.35          # distancia pulgar-indice / palma por debajo de la cual es un "OK"
PULGAR_FUERA = 1.0      # cociente del pulgar (rt) por encima del cual esta estirado (gesto llamada)
PULGAR_VERTICAL = 0.70  # el pulgar debe apuntar arriba/abajo al menos esta fraccion de la palma (en imagen)

# --- Filtro One Euro (suave quieto, rapido en movimiento): (corte minimo Hz, beta) ---
FILTRO = {"x": (0.8, 8.0), "y": (0.8, 8.0), "t": (1.5, 2.0),
          "ri": (2.0, 1.0), "rm": (2.0, 1.0), "ra": (2.0, 1.0), "rp": (2.0, 1.0),
          "q4": (2.0, 1.0), "ang": (1.0, 0.05), "okd": (2.0, 1.0), "rt": (2.0, 1.0), "pv": (2.0, 1.0), "ph": (2.0, 1.0)}

# --- Seguridad: grados maximos por cuadro de cada servo y frecuencia de envio ---
MAX_PASO = {"A": 4.0, "B": 4.0, "C": 5.0, "D": 12.0}
INTERVALO_ENVIO = 0.05
INTERVALO_LATIDO = 0.1     # PING al ESP32 desde un hilo propio (el watchdog del firmware es de 1.5 s)

# --- Confianza minima de MediaPipe (anti falsos positivos) ---
CONF_DETECCION = 0.7      # deteccion inicial de la mano
CONF_PRESENCIA = 0.7      # presencia de la mano en cada cuadro
CONF_SEGUIMIENTO = 0.6    # seguimiento entre cuadros
CONF_LADO_MIN = 0.80      # puntaje minimo de la clasificacion izquierda/derecha; si es menor se ignora el cuadro

CONEXIONES = [
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
]
FONT = cv2.FONT_HERSHEY_SIMPLEX


def limitar(v, lo, hi):
    return max(lo, min(hi, v))


def cargar_limites():
    limites = dict(LIMITES_DEFECTO)
    usados = []
    if USAR_CALIBRACION and ARCHIVO_CALIB.exists():
        try:
            datos = json.loads(ARCHIVO_CALIB.read_text(encoding="utf-8"))
            for n in NOMBRES:
                lo, hi = datos.get(n, {}).get("usar_min"), datos.get(n, {}).get("usar_max")
                if lo is not None and hi is not None and lo < hi:
                    limites[n] = (lo, hi)
                    usados.append(n)
        except (OSError, ValueError):
            print("Aviso: no pude leer calibracion.json, uso limites por defecto.")
    print("Limites:", {n: limites[n] for n in NOMBRES},
          "(de calibracion.json: " + (", ".join(usados) or "ninguno") + ")")
    return limites


# ---- medicion de la mano ----
def medir(lm, lm3d, w, h):
    """Mide la mano. lm: puntos de imagen (x,y en 0..1). lm3d: puntos 'world' (metros) o None."""
    centro = [lm[i] for i in (0, 5, 9, 13, 17)]
    x = sum(p.x for p in centro) / len(centro)
    y = sum(p.y for p in centro) / len(centro)

    if lm3d is not None:
        def dist(i, j):
            a, b = lm3d[i], lm3d[j]
            return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
    else:                                   # sin 3D: aproximacion en la imagen
        def dist(i, j):
            return math.hypot((lm[i].x - lm[j].x) * w, (lm[i].y - lm[j].y) * h)

    palma = max(dist(0, 9), 1e-6)
    ratios = {}
    for clave, punta, base in (("ri", 8, 5), ("rm", 12, 9), ("ra", 16, 13), ("rp", 20, 17)):
        ratios[clave] = dist(0, punta) / max(dist(0, base), 1e-6)
    q4 = sum(ratios.values()) / 4
    rt = dist(4, 17) / palma
    t_dedos = limitar((q4 - Q_CERRADO) / (Q_ABIERTO - Q_CERRADO), 0.0, 1.0)
    t_pulgar = limitar((rt - T_CERRADO) / (T_ABIERTO - T_CERRADO), 0.0, 1.0)
    t = 0.8 * t_dedos + 0.2 * t_pulgar          # 0 = puno cerrado, 1 = mano abierta
    # direccion del pulgar en la imagen (base del pulgar -> punta), relativa a la palma
    palma_img = max(math.hypot((lm[0].x - lm[9].x) * w, (lm[0].y - lm[9].y) * h), 1e-6)
    pv = (lm[4].y - lm[2].y) * h / palma_img     # negativo = pulgar hacia arriba
    ph = (lm[4].x - lm[2].x) * w / palma_img
    # giro de la mano en el plano de la imagen: 0 = dedos hacia arriba, + = girada hacia la derecha
    ang = math.degrees(math.atan2((lm[9].x - lm[0].x) * w, -(lm[9].y - lm[0].y) * h))
    okd = dist(4, 8) / palma
    return {"x": x, "y": y, "t": t, "q4": q4, "rt": rt, "pv": pv, "ph": ph, "ang": ang, "okd": okd, **ratios}


class UnEuro:
    """Filtro One Euro: mucho suavizado cuando la mano esta quieta, poco cuando se mueve rapido."""

    def __init__(self, corte_min, beta, corte_d=1.0):
        self.fc, self.beta, self.fd = corte_min, beta, corte_d
        self.t = self.x = self.dx = None

    @staticmethod
    def _alfa(corte, dt):
        tau = 1.0 / (2 * math.pi * corte)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, valor, t):
        if self.t is None:
            self.t, self.x, self.dx = t, valor, 0.0
            return valor
        dt = max(t - self.t, 1e-3)
        d = (valor - self.x) / dt
        self.dx += self._alfa(self.fd, dt) * (d - self.dx)
        self.x += self._alfa(self.fc + self.beta * abs(self.dx), dt) * (valor - self.x)
        self.t = t
        return self.x


class Filtro:
    """Filtra todas las magnitudes y descarta saltos imposibles de la posicion (errores de deteccion)."""

    def __init__(self):
        self.reiniciar()

    def reiniciar(self):
        self.f = {k: UnEuro(*p) for k, p in FILTRO.items()}
        self.ultimo = None
        self.rechazos = 0

    def __call__(self, m, t):
        if self.ultimo is not None and self.rechazos < 3:
            if abs(m["x"] - self.ultimo["x"]) > MAX_SALTO or abs(m["y"] - self.ultimo["y"]) > MAX_SALTO:
                self.rechazos += 1
                m = dict(m, x=self.ultimo["x"], y=self.ultimo["y"])
            else:
                self.rechazos = 0
        else:
            self.rechazos = 0
        salida = {k: self.f[k](v, t) if k in self.f else v for k, v in m.items()}
        self.ultimo = salida
        return dict(salida)


def clasificar(ext, pv=0.0, ph=0.0, okd=9.0, rt=0.0):
    i, m, a, p = ext
    if m and a and p and okd < OK_DIST:
        return "ok"
    if i and m and a and p:
        return "abierta"
    if not (i or m or a or p):
        if abs(pv) > PULGAR_VERTICAL and abs(pv) > 1.5 * abs(ph):
            return "pulgar_arriba" if pv < 0 else "pulgar_abajo"
        return "puno"
    if i and m and not a and not p:
        return "paz"
    if i and p and not m and not a:
        return "cuernos"
    if p and not (i or m or a) and rt > PULGAR_FUERA:
        return "llamada"
    return "otro"


class Gestos:
    """Estado de cada dedo con histeresis + confirmacion de varios cuadros seguidos."""

    def __init__(self):
        self.reiniciar()

    def reiniciar(self):
        self.ext = None
        self.cand = None
        self.n = 0
        self.estable = "otro"
        self.evento = None

    def actualizar(self, m):
        valores = [m["ri"], m["rm"], m["ra"], m["rp"]]
        if self.ext is None:
            self.ext = [v > (EXT_ON + EXT_OFF) / 2 for v in valores]
        for k, v in enumerate(valores):
            if v > EXT_ON:
                self.ext[k] = True
            elif v < EXT_OFF:
                self.ext[k] = False
        nombre = clasificar(self.ext, m.get("pv", 0.0), m.get("ph", 0.0), m.get("okd", 9.0), m.get("rt", 0.0))
        if nombre == self.cand:
            self.n += 1
        else:
            self.cand, self.n = nombre, 1
        previo = self.estable
        self.evento = None
        if self.n >= FRAMES_CONFIRMAR.get(nombre, 4):
            self.estable = nombre
            if nombre != previo and nombre in ACCIONES:
                self.evento = nombre            # se dispara una sola vez por gesto
        return self.estable


# ---- control del brazo ----
class Latido(threading.Thread):
    """Hilo que manda PING al ESP32 aunque el video se atrase o no haya mano: mantiene vivo el enlace."""

    def __init__(self, ctl):
        super().__init__(daemon=True)
        self.ctl = ctl
        self._alto = threading.Event()

    def run(self):
        while not self._alto.is_set():
            self.ctl.enviar("PING")
            self._alto.wait(INTERVALO_LATIDO)

    def detener(self):
        self._alto.set()
        self.join(timeout=1.0)


class Control:
    def __init__(self, ser, limites):
        self.ser = ser
        self._lock_envio = threading.Lock()
        self.lim = limites
        self.neutro = {n: limitar(NEUTRO[n], *limites[n]) for n in NOMBRES}
        self.activo = False
        self.soltado = False
        self.fino = False
        self.gesto = "otro"
        self.t_envio = 0.0
        self.ancla = None
        self.diag = None
        self.aviso = ('', 0.0)
        self.seguro = False            # paro SAFE (tecla X)
        self.estado_fw = "?"           # ultimo estado informado por el ESP32
        self.datos = None              # ultima telemetria DATA:
        self.reiniciar_angulos()

    def reiniciar_angulos(self):
        self.obj = {n: float(self.neutro[n]) for n in NOMBRES}
        self.cmd = dict(self.obj)
        self.enviados = {n: int(round(self.cmd[n])) for n in NOMBRES}
        self.ancla = None

    def enviar(self, texto):
        with self._lock_envio:                 # el hilo del latido tambien envia
            self.ser.write((texto + "\n").encode())

    def leer(self):
        while self.ser.in_waiting:
            linea = self.ser.readline().decode(errors="ignore").strip()
            if not linea:
                continue
            if linea.startswith("STATE:"):
                self.estado_fw = linea[6:]
                if self.estado_fw in ("SAFE", "ERROR"):
                    self.activo = False
                    self.ancla = None
            elif linea.startswith("DATA:"):
                try:
                    self.datos = [int(v) for v in linea[5:].split(",")]
                except ValueError:
                    pass

    def trabajo(self):
        self.enviar("RESET")
        self.enviar("TRABAJO")
        self.activo = False
        self.soltado = False
        self.reiniciar_angulos()

    def montaje(self):
        self.enviar("RESET")
        self.enviar("MONTAJE")
        self.activo = False
        self.soltado = False
        self.obj = {n: 90.0 for n in NOMBRES}
        self.cmd = dict(self.obj)
        self.enviados = {n: 90 for n in NOMBRES}
        self.ancla = None

    def paro(self):
        """Paro de emergencia (tecla X): SAFE en el ESP32. Los servos se congelan SOSTENIENDO su posicion."""
        self.enviar("SAFE")
        self.activo = False
        self.seguro = True
        self.ancla = None

    def soltar(self):
        """Tecla L: suelta los servos (el brazo puede caer)."""
        self.enviar("REL")
        self.activo = False
        self.soltado = True

    def activar(self):
        """Rearma y activa: RESET -> START -> TRABAJO."""
        self.soltado = False
        self.seguro = False
        self.reiniciar_angulos()
        self.enviar("RESET")
        self.enviar("START")
        self.enviar("TRABAJO")
        self.activo = True

    def avisar(self, texto):
        self.aviso = (texto, time.monotonic() + 1.8)

    def ir_a_trabajo(self):
        """Vuelve a la posicion de trabajo sin desactivar el control."""
        if self.soltado or self.seguro:
            return
        self.enviar("TRABAJO")
        self.obj = {n: float(self.neutro[n]) for n in NOMBRES}
        self.cmd = dict(self.obj)
        self.enviados = {n: int(round(self.cmd[n])) for n in NOMBRES}
        self.ancla = None

    def accion(self, evento):
        if evento == "ok":
            self.alternar_fino()
            self.avisar("MODO FINO " + ("ACTIVADO" if self.fino else "DESACTIVADO"))
        elif evento == "cuernos":
            if not (self.soltado or self.seguro):
                self.alternar()
                self.avisar("CONTROL ACTIVO" if self.activo else "CONTROL EN PAUSA")
        elif evento == "llamada":
            self.ir_a_trabajo()
            self.avisar("POSICION DE TRABAJO")

    def alternar(self):
        if self.soltado or self.seguro:
            return
        self.activo = not self.activo
        self.ancla = None

    def empujar_c(self, grados):
        """Mueve el antebrazo con el teclado (grados negativos = sube)."""
        if self.activo:
            lo, hi = self.lim["C"]
            self.obj["C"] = limitar(self.obj["C"] + grados, lo, hi)

    def alternar_fino(self):
        self.fino = not self.fino

    def _sumar_eje(self, n, pos, ancla, ganancia, signo, banda=BANDA):
        """Suma movimiento relativo solo cuando la mano se desplaza mas que la banda."""
        d = pos - ancla
        if abs(d) < banda:
            return ancla
        k = FACTOR_FINO if self.fino else 1.0
        self.obj[n] += signo * d * ganancia * k
        return pos

    def actualizar(self, medida, gesto, ahora):
        self.gesto = gesto
        if not self.activo:
            return
        if medida is None:
            self.ancla = None                     # sin mano = embrague
        elif gesto in ("abierta", "puno"):
            if self.ancla is None:
                self.ancla = [medida["x"], medida["y"], medida["ang"]]
            else:
                # camara en espejo: mano a la izquierda (dx<0) -> A+ ; mano abajo (dy>0) -> B+ (bajar)
                self.ancla[0] = self._sumar_eje("A", medida["x"], self.ancla[0], GANANCIA_A, -1)
                self.ancla[1] = self._sumar_eje("B", medida["y"], self.ancla[1], GANANCIA_B, +1)
                # perilla: girar a la derecha (angulo +) -> C+ (baja); a la izquierda -> sube
                self.ancla[2] = self._sumar_eje("C", medida["ang"], self.ancla[2], GANANCIA_C, +1, BANDA_ANG)
            lo, hi = self.lim["D"]
            t = limitar((medida["t"] - PINZA_T_CERRADA) / (PINZA_T_ABIERTA - PINZA_T_CERRADA), 0.0, 1.0)
            self.obj["D"] = lo + t * (hi - lo)    # puno -> cerrada, mano abierta -> abierta
        else:
            self.ancla = None                     # paz / otro = embrague
            if gesto == "pulgar_arriba":
                self.obj["C"] -= JOG_C            # C- sube
            elif gesto == "pulgar_abajo":
                self.obj["C"] += JOG_C            # C+ baja

        for n in NOMBRES:
            lo, hi = self.lim[n]
            self.obj[n] = limitar(self.obj[n], lo, hi)
            delta = self.obj[n] - self.cmd[n]
            self.cmd[n] += limitar(delta, -MAX_PASO[n], MAX_PASO[n])

        if ahora - self.t_envio < INTERVALO_ENVIO:
            return
        self.t_envio = ahora
        for n in NOMBRES:
            lo, hi = self.lim[n]
            ang = int(round(limitar(self.cmd[n], lo, hi)))
            if ang != self.enviados[n]:
                self.enviar(f"{n},{ang}")
                self.enviados[n] = ang


# ---- interfaz ----
def dibujar_mano(frame, lm):
    h, w = frame.shape[:2]
    pts = [(int(p.x * w), int(p.y * h)) for p in lm]
    for a, b in CONEXIONES:
        cv2.line(frame, pts[a], pts[b], (255, 255, 0), 2)
    for p in pts:
        cv2.circle(frame, p, 4, (0, 255, 0), -1)


NOMBRE_GESTO = {"abierta": "MANO ABIERTA (mueve A+B, pinza abre)", "puno": "PUNO (mueve A+B, pinza cierra)",
                "paz": "PAZ = EMBRAGUE", "pulgar_arriba": "PULGAR ARRIBA = antebrazo SUBE",
                "pulgar_abajo": "PULGAR ABAJO = antebrazo BAJA", "ok": "OK = modo fino",
                "cuernos": "CUERNOS = pausa / reanudar", "llamada": "LLAMADA = posicion de trabajo", "otro": "gesto no valido = EMBRAGUE"}


def dibujar_hud(frame, ctl, medida):
    h, w = frame.shape[:2]
    if ctl.estado_fw == "SIN ENLACE":
        estado, color = "SIN ENLACE Wi-Fi: reintentando... (el brazo baja solo a reposo)", (0, 0, 255)
    elif ctl.estado_fw == "ERROR":
        estado, color = "ERROR DE ENLACE: el brazo bajo a reposo. Pulsa C para rearmar", (0, 0, 255)
    elif ctl.seguro or ctl.estado_fw == "SAFE":
        estado, color = "PARO DE EMERGENCIA (SAFE): servos congelados. Pulsa C para rearmar", (0, 0, 255)
    elif ctl.soltado:
        estado, color = "SERVOS SOLTADOS (pulsa C para volver)", (0, 165, 255)
    elif ctl.activo:
        estado, color = "ACTIVO" + ("  [MODO FINO]" if ctl.fino else ""), (0, 255, 0)
    else:
        estado, color = "PAUSADO - pulsa C", (0, 0, 255)
    cv2.putText(frame, estado, (15, 30), FONT, 0.75, color, 2)

    y = 62
    for n in NOMBRES:
        cv2.putText(frame, f"{n} {DESCRIPCION[n]:<14} {int(round(ctl.cmd[n])):3d}",
                    (15, y), FONT, 0.55, (255, 255, 255), 1)
        y += 22

    if medida is None:
        cv2.putText(frame, "SIN MANO (embrague: el brazo se queda quieto)", (15, y + 8), FONT, 0.55, (0, 0, 255), 2)
    else:
        mueve = ctl.gesto in ("abierta", "puno")
        cv2.putText(frame, NOMBRE_GESTO.get(ctl.gesto, ctl.gesto), (15, y + 8), FONT, 0.6,
                    (0, 255, 0) if mueve else (0, 200, 255), 2)
        cv2.putText(frame, f"apertura={medida['t']:.2f}  dedos q={medida['q4']:.2f}  pulgar={medida['rt']:.2f}",
                    (15, y + 32), FONT, 0.5, (200, 200, 200), 1)
        if ctl.diag:
            ext, cand, n = ctl.diag
            if cand in ACCIONES and cand != ctl.gesto:
                barra = int(200 * min(n / FRAMES_CONFIRMAR[cand], 1.0))
                cv2.rectangle(frame, (15, y + 86), (215, y + 98), (90, 90, 90), 1)
                cv2.rectangle(frame, (15, y + 86), (15 + barra, y + 98), (0, 255, 255), -1)
            cv2.putText(frame, f"dedos I M A P: {' '.join('1' if e else '0' for e in ext)}  viendo: {cand} x{n}",
                        (15, y + 54), FONT, 0.5, (0, 255, 255), 1)
            cv2.putText(frame, f"cocientes I={medida['ri']:.2f} M={medida['rm']:.2f} A={medida['ra']:.2f} P={medida['rp']:.2f}  pulgar vert={medida['pv']:.2f}  giro={medida['ang']:+.0f} grados",
                        (15, y + 74), FONT, 0.5, (0, 255, 255), 1)
    real = "" if ctl.datos is None else "  real " + " ".join(f"{n}={v}" for n, v in zip(NOMBRES, ctl.datos))
    cv2.putText(frame, f"ESP32: {ctl.estado_fw}{real}", (w - 430, 30), FONT, 0.55, (255, 255, 255), 1)
    texto, hasta = ctl.aviso
    if texto and time.monotonic() < hasta:
        cv2.putText(frame, texto, (w // 2 - 160, 60), FONT, 0.9, (0, 255, 255), 3)
    cv2.putText(frame, "OK: modo fino | CUERNOS: pausa | LLAMADA: posicion de trabajo",
                (15, h - 34), FONT, 0.5, (0, 255, 255), 1)
    cv2.putText(frame, "C: activar | ESPACIO: pausa | F: fino | U/J: C | M: mont. | T: trab. | X: PARO | L: soltar | ESC",
                (15, h - 12), FONT, 0.45, (255, 255, 255), 1)
    return frame


def asegurar_modelo():
    if MODEL_PATH.exists():
        return
    MODEL_PATH.parent.mkdir(exist_ok=True)
    print(f"No encuentro {MODEL_PATH.name}; intentando descargarlo...")
    try:
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Modelo descargado en", MODEL_PATH)
    except Exception as e:
        if MODEL_PATH.exists():
            MODEL_PATH.unlink()
        raise SystemExit(
            f"No pude descargar el modelo ({e}).\n"
            f"Copia tu archivo hand_landmarker.task (de tu proyecto de gestos) a:\n  {MODEL_PATH}"
        )


def abrir_camara(preferida):
    """Abre la camara con AVFoundation (Mac): prueba la preferida y luego las demas."""
    for index in [preferida] + [i for i in (0, 1, 2) if i != preferida]:
        cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                print(f"Camara abierta en el indice {index}")
                return cap
        cap.release()
    raise SystemExit("No se pudo abrir la camara. Revisa el permiso en Ajustes del Sistema > Privacidad > Camara.")


def main():
    import mediapipe as mp

    asegurar_modelo()
    limites = cargar_limites()

    print("\n=== SELECCIONA LA CAMARA ===")
    print("1: Camara del iPhone (Continuity Camera)")
    print("2: Camara integrada de la Mac")
    opcion = input("Elige una opcion (1 o 2) [Default: 2]: ").strip()
    cap = abrir_camara(0 if opcion == "1" else 1)

    with abrir_enlace() as ser:
        ctl = Control(ser, limites)
        ctl.trabajo()
        latido = Latido(ctl)
        latido.start()

        BaseOptions = mp.tasks.BaseOptions
        opciones = mp.tasks.vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(MODEL_PATH),
                                     delegate=BaseOptions.Delegate.CPU),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=CONF_DETECCION,
            min_hand_presence_confidence=CONF_PRESENCIA,
            min_tracking_confidence=CONF_SEGUIMIENTO,
        )
        filtro = Filtro()
        gestos = Gestos()
        ultimo_ts = 0
        t_reintento = 0.0

        with mp.tasks.vision.HandLandmarker.create_from_options(opciones) as detector:
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                frame = cv2.flip(frame, 1)          # espejo
                h, w = frame.shape[:2]

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                ts = max(int(time.monotonic() * 1000), ultimo_ts + 1)
                ultimo_ts = ts
                res = detector.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)

                ahora = time.monotonic()
                medida = None
                confiable = bool(res.hand_landmarks)
                if confiable and res.handedness:
                    confiable = res.handedness[0][0].score >= CONF_LADO_MIN
                if confiable:
                    lm = res.hand_landmarks[0]
                    lm3d = res.hand_world_landmarks[0] if res.hand_world_landmarks else None
                    dibujar_mano(frame, lm)
                    medida = filtro(medir(lm, lm3d, w, h), ahora)
                    gesto = gestos.actualizar(medida)
                    ctl.diag = (list(gestos.ext), gestos.cand, gestos.n)
                else:
                    filtro.reiniciar()
                    gestos.reiniciar()
                    gesto = "otro"
                    ctl.diag = None

                if medida is not None and gestos.evento:
                    ctl.accion(gestos.evento)
                ctl.leer()
                ctl.actualizar(medida, gesto, ahora)
                if getattr(ser, "cerrado", False):
                    if ctl.estado_fw != "SIN ENLACE":
                        print(f"Enlace perdido ({ser.motivo}). El brazo baja solo a reposo; reintentando...")
                        ctl.estado_fw = "SIN ENLACE"
                        ctl.activo = False
                        ctl.ancla = None
                    if ahora - t_reintento > 1.0:
                        t_reintento = ahora
                        if ser.reconectar():
                            print("Enlace recuperado. Pulsa C para rearmar.")
                            ctl.estado_fw = "?"
                cv2.imshow("Brazo por gestos", dibujar_hud(frame, ctl, medida))

                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break
                tecla = chr(k).lower() if k != 255 else ""
                if tecla == "c":
                    ctl.activar()
                elif tecla == " ":
                    ctl.alternar()
                elif tecla == "u":
                    ctl.empujar_c(-5)
                elif tecla == "j":
                    ctl.empujar_c(+5)
                elif tecla == "f":
                    ctl.alternar_fino()
                elif tecla == "m":
                    ctl.montaje()
                elif tecla == "t":
                    ctl.trabajo()
                elif tecla == "x":
                    ctl.paro()
                elif tecla == "l":
                    ctl.soltar()

        latido.detener()
        ctl.enviar("STOP")

    cap.release()
    cv2.destroyAllWindows()
    print("Los servos quedaron sosteniendo su posicion (ESP32 en IDLE). Baja el brazo con la mano antes de apagar el interruptor.")


if __name__ == "__main__":
    main()
