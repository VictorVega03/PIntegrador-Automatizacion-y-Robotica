import math
import random
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import main as g

class P:
    def __init__(s, x, y, z=0.0): s.x, s.y, s.z = x, y, z

def mano(cx=0.5, cy=0.5, dedos=(1.9, 1.9, 1.9, 1.9), pulgar=1.2, dirp='lado', rot=0.0, ok=False, w=1280, h=720, ruido=0.0, rng=None):
    """dedos = cociente punta/base (1.9 extendido, 0.8 doblado). Devuelve (img, world, w, h)."""
    palma = 0.09                                  # metros
    k = 0.2 * h / h / palma                       # escala metros -> fraccion de alto de imagen
    xs = {5: -0.02, 9: 0.0, 13: 0.02, 17: 0.04}
    world = [P(0, 0, 0) for _ in range(21)]
    for i, x in xs.items():
        world[i] = P(x, -palma, 0)
    for (tip, base), r in zip(((8, 5), (12, 9), (16, 13), (20, 17)), dedos):
        world[tip] = P(xs[base], -palma * r, 0)
    world[4] = P(world[17].x - pulgar * palma, world[17].y, 0)
    img = []
    for p in world:
        jx = (rng.gauss(0, ruido) if rng else 0); jy = (rng.gauss(0, ruido) if rng else 0)
        img.append(P(cx + p.x * k * h / w + jx, cy + p.y * k + jy))
    if ok:
        world[4] = P(world[8].x + 0.005, world[8].y + 0.005, 0)
    img[2] = P(cx - 0.02, cy)
    dx, dy = {'lado': (-0.12 * h / w, 0.0), 'arriba': (0.0, -0.18), 'abajo': (0.0, 0.18)}[dirp]
    img[4] = P(cx - 0.02 + dx, cy + dy)
    if rot:
        th = math.radians(rot); ox, oy = img[0].x * w, img[0].y * h
        for p in img:
            px, py = p.x * w - ox, p.y * h - oy
            p.x = (ox + px * math.cos(th) - py * math.sin(th)) / w
            p.y = (oy + px * math.sin(th) + py * math.cos(th)) / h
    return img, world, w, h

class FakeSer:
    def __init__(s): s.sent = []; s.in_waiting = 0
    def write(s, b): s.sent.append(b.decode().strip())

ABIERTA = (1.9,) * 4; PUNO = (0.85,) * 4
PAZ = (1.9, 1.9, 0.85, 0.85)

def medida(**kw):
    img, wor, w, h = mano(**kw)
    return g.medir(img, wor, w, h)

# 1) medicion
ma, mp_ = medida(dedos=ABIERTA, pulgar=1.2), medida(dedos=PUNO, pulgar=0.4)
assert ma["t"] > 0.9 and mp_["t"] < 0.1, (ma["t"], mp_["t"])

# 2) clasificacion con confirmacion
def clasif(dedos, n=10):
    G = g.Gestos(); r = None
    for _ in range(n): r = G.actualizar(medida(dedos=dedos))
    return r
assert clasif(ABIERTA) == "abierta" and clasif(PUNO) == "puno" and clasif(PAZ) == "paz"
def clasif2(dedos, dirp, n=10):
    G = g.Gestos(); r = None
    for _ in range(n): r = G.actualizar(medida(dedos=dedos, pulgar=1.2, dirp=dirp))
    return r
assert clasif2(PUNO, "arriba") == "pulgar_arriba" and clasif2(PUNO, "abajo") == "pulgar_abajo"
assert clasif2(PUNO, "lado") == "puno"
G = g.Gestos()
for _ in range(10): G.actualizar(medida(dedos=ABIERTA))
for _ in range(2): G.actualizar(medida(dedos=PUNO, dirp="arriba"))   # 2 cuadros no bastan
assert G.estable == "abierta"

# 3) simulacion de sesion
lim = {"A": (14, 172), "B": (54, 172), "C": (8, 172), "D": (8, 98)}
ser = FakeSer(); c = g.Control(ser, lim)
c.activar(); assert ser.sent[-1] == "TRABAJO" and c.activo
tt = [100.0]
def paso(m, gesto, n=1):
    for _ in range(n):
        tt[0] += 0.04
        c.actualizar(m, gesto, tt[0])
    return {k: round(v) for k, v in c.cmd.items()}

paso(medida(dedos=ABIERTA), "abierta", 3)
a0 = dict(c.cmd)
# mover mano a la derecha 0.2 de imagen -> A baja ~ 160*0.2 = 32
for i in range(1, 21): paso(medida(cx=0.5 + 0.01 * i, dedos=ABIERTA), "abierta")
assert 25 < a0["A"] - c.cmd["A"] < 40, (a0, c.cmd)
# mano abajo -> B sube de angulo
b0 = c.cmd["B"]
for i in range(1, 21): paso(medida(cx=0.7, cy=0.5 + 0.01 * i, dedos=ABIERTA), "abierta")
assert 25 < c.cmd["B"] - b0 < 45, c.cmd
# A y B a la vez
a1, b1 = c.cmd["A"], c.cmd["B"]
for i in range(1, 11): paso(medida(cx=0.7 - 0.01 * i, cy=0.7 - 0.01 * i, dedos=ABIERTA), "abierta")
assert c.cmd["A"] > a1 and c.cmd["B"] < b1
# puno -> pinza cierra, abierta -> abre
paso(medida(cx=0.6, cy=0.6, dedos=PUNO, pulgar=0.4), "puno", 20); assert round(c.cmd["D"]) == 8, c.cmd
paso(medida(cx=0.6, cy=0.6, dedos=ABIERTA), "abierta", 20); assert round(c.cmd["D"]) == 98, c.cmd
# embrague: paz no mueve nada aunque la mano se mueva, y al volver no hay salto
snap = dict(c.cmd)
for i in range(30): paso(medida(cx=0.2 + 0.01 * i, cy=0.3, dedos=PAZ), "paz")
assert {k: round(v) for k, v in c.cmd.items()} == {k: round(v) for k, v in snap.items()}
paso(medida(cx=0.9, cy=0.9, dedos=ABIERTA), "abierta", 5)
assert abs(c.cmd["A"] - snap["A"]) < 2 and abs(c.cmd["B"] - snap["B"]) < 2, (c.cmd, snap)
# jog de C
c0 = c.cmd["C"]; paso(medida(dedos=PUNO, dirp="arriba"), "pulgar_arriba", 10); assert c.cmd["C"] < c0
c1 = c.cmd["C"]; d_ant = c.cmd["D"]; paso(medida(dedos=PUNO, dirp="abajo"), "pulgar_abajo", 20); assert c.cmd["C"] > c1
assert abs(c.cmd["D"] - d_ant) < 1          # la pinza no cambia mientras se mueve C
# sin mano: nada se mueve, y re-engancha sin salto
snap = dict(c.cmd); paso(None, "otro", 5); assert snap == c.cmd
paso(medida(cx=0.3, cy=0.3, dedos=ABIERTA), "abierta", 3)
assert abs(c.cmd["A"] - snap["A"]) < 2
# nunca fuera de limites
for linea in ser.sent:
    if "," in linea:
        n, ang = linea.split(","); lo, hi = lim[n]; assert lo <= int(ang) <= hi, linea

# 3b) perilla: girar a la derecha -> C baja; a la izquierda -> C sube; quieto no mueve
assert abs(medida(rot=30)["ang"] - 30) < 1 and abs(medida(rot=-20)["ang"] + 20) < 1
ser3 = FakeSer(); c = g.Control(ser3, lim); c.activar(); tt = [300.0]
paso(medida(dedos=ABIERTA), "abierta", 3)
c0 = c.cmd["C"]
for i in range(1, 31): paso(medida(rot=float(i)), "abierta")           # +30 grados
assert 35 < c.cmd["C"] - c0 < 55, (c0, c.cmd["C"])                    # ~ 30*1.6 = 48
c1 = c.cmd["C"]
for i in range(1, 21): paso(medida(rot=30.0 - i), "abierta")           # de vuelta 20 grados
assert 20 < c1 - c.cmd["C"] < 40, c.cmd
c2 = c.cmd["C"]; paso(medida(rot=10.4), "abierta", 15)                 # temblor < banda: no mueve
assert abs(c.cmd["C"] - c2) < 1
for linea in ser3.sent:
    if "," in linea:
        n, ang = linea.split(","); lo, hi = lim[n]; assert lo <= int(ang) <= hi, linea

# 3c) gestos de accion
OK = dict(dedos=(1.4, 1.9, 1.9, 1.9), ok=True)
CUERNOS = (1.9, 0.85, 0.85, 1.9); LLAMADA = (0.85, 0.85, 0.85, 1.9)
def clas3(n=20, **kw):
    G = g.Gestos(); ev = []
    for _ in range(n):
        G.actualizar(medida(**kw)); ev.append(G.evento)
    return G.estable, [e for e in ev if e]
assert clas3(**OK) == ("ok", ["ok"]), clas3(**OK)
assert clas3(dedos=CUERNOS) == ("cuernos", ["cuernos"])
assert clas3(dedos=LLAMADA, pulgar=1.2) == ("llamada", ["llamada"])
assert clas3(dedos=LLAMADA, pulgar=0.4)[0] != "llamada"       # sin pulgar fuera es un puno con menique
assert clas3(dedos=ABIERTA) == ("abierta", [])
assert clas3(dedos=PUNO, pulgar=0.4) == ("puno", [])
assert clas3(n=8, **OK)[0] != "ok"                             # hay que mantenerlo ~0.5 s
ser4 = FakeSer(); c = g.Control(ser4, lim); c.activar()
c.accion("ok"); assert c.fino; c.accion("ok"); assert not c.fino
c.accion("cuernos"); assert not c.activo; c.accion("cuernos"); assert c.activo
tt = [400.0]; paso(medida(dedos=ABIERTA), "abierta", 3)
for i in range(1, 11): paso(medida(cx=0.5 + 0.01 * i), "abierta")
assert abs(c.cmd["A"] - 90) > 5
c.accion("llamada"); assert ser4.sent[-1] == "TRABAJO" and c.activo and round(c.cmd["A"]) == 90
c.soltar(); n_env = len(ser4.sent); c.accion("llamada"); assert len(ser4.sent) == n_env   # soltado: no reactiva

# 3d) FSM del lado Python: paro, estados, telemetria, latido
class FakeSerLineas:
    def __init__(s): s.sent = []; s.cola = []
    def write(s, b): s.sent.append(b.decode().strip())
    @property
    def in_waiting(s): return len(s.cola)
    def readline(s): return (s.cola.pop(0) + "\n").encode()
sf = FakeSerLineas(); c = g.Control(sf, lim); c.activar()
assert sf.sent[-3:] == ["RESET", "START", "TRABAJO"] and c.activo
sf.cola += ["ACK:ACTIVE", "STATE:ACTIVE", "DATA:90,120,60,90", "PONG", "OK A 90"]; c.leer()
assert c.estado_fw == "ACTIVE" and c.datos == [90, 120, 60, 90]
import time as _t
hl = g.Latido(c); hl.start(); _t.sleep(0.55)          # el hilo manda PING aunque el programa principal este "colgado"
hl.detener(); n_ping = sf.sent.count("PING"); assert n_ping >= 4, n_ping
_t.sleep(0.3); assert sf.sent.count("PING") == n_ping    # tras detenerlo ya no manda
c.paro(); assert sf.sent[-1] == "SAFE" and not c.activo and c.seguro
n = len(sf.sent); c.accion("llamada"); c.accion("cuernos"); c.alternar(); assert len(sf.sent) == n and not c.activo
c.activar(); assert c.activo and not c.seguro and sf.sent[-3:] == ["RESET", "START", "TRABAJO"]
sf.cola += ["STATE:ERROR"]; c.leer(); assert not c.activo and c.estado_fw == "ERROR"
c.activar(); sf.cola += ["STATE:SAFE"]; c.leer(); assert not c.activo
c.soltar(); assert sf.sent[-1] == "REL" and c.soltado

# 3e) EnlaceTCP contra un servidor local que imita al ESP32
import socket, threading, time
from enlace import EnlaceTCP
srv = socket.socket(); srv.bind(("127.0.0.1", 0)); srv.listen(1); puerto = srv.getsockname()[1]
recibido = []
def servidor():
    cn, _ = srv.accept(); cn.sendall(b"READY\nSTATE:IDLE\n")
    datos = b""
    while True:
        d = cn.recv(1024)
        if not d: break
        datos += d
        while b"\n" in datos:
            l, datos = datos.split(b"\n", 1); recibido.append(l.decode())
            cn.sendall(b"PONG\n" if l == b"PING" else b"OK " + l + b"\n")
    cn.close()
th = threading.Thread(target=servidor, daemon=True); th.start()
e = EnlaceTCP("127.0.0.1", puerto)
time.sleep(0.2); assert e.in_waiting == 2 and e.readline().decode().strip() == "READY" and e.readline().decode().strip() == "STATE:IDLE"
e.write(b"PING\nA,90\n"); time.sleep(0.2)
assert e.in_waiting == 2 and e.readline() == b"PONG\n" and e.readline() == b"OK A,90\n" and e.readline() == b""
assert recibido == ["PING", "A,90"]
e.close(); th.join(timeout=2)
import threading as _th; e2 = EnlaceTCP.__new__(EnlaceTCP); e2._lock = _th.RLock(); e2.cerrado = True; assert e2.write(b"x") == 0   # enlace caido no lanza excepcion

# 4) ruido: mano quieta con jitter -> el brazo casi no se mueve
rng = random.Random(1)
ser2 = FakeSer(); c = g.Control(ser2, lim); c.activar(); F = g.Filtro()
t = 200.0
for i in range(150):
    t += 0.033
    img, wor, w, h = mano(dedos=ABIERTA, ruido=0.002, rng=rng)
    m = F(g.medir(img, wor, w, h), t)
    c.actualizar(m, "abierta", t)
assert abs(c.cmd["A"] - 90) < 4 and abs(c.cmd["B"] - 120) < 4, c.cmd
# salto espurio de deteccion (pico): se ignora
F = g.Filtro(); m0 = F(medida(), 1.0); m1 = F(medida(cx=0.9), 1.03)
assert abs(m1["x"] - m0["x"]) < 0.01, (m0["x"], m1["x"])

# 5) HUD
import cv2, numpy as np
frame = np.full((720, 1280, 3), 40, np.uint8)
img, wor, w, h = mano(); g.dibujar_mano(frame, img)
c.gesto = "abierta"; g.dibujar_hud(frame, c, medida())
cv2.imwrite(str(Path(tempfile.gettempdir()) / "hud5.png"), cv2.resize(frame, (960, 540)))
print("TODAS LAS PRUEBAS OK")
