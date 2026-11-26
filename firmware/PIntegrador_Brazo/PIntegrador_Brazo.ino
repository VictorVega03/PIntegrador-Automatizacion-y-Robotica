/*
 * PIntegrador_Brazo: firmware del brazo eArm (ESP32-C3) controlado por gestos desde Python.
 * Placa: "ESP32C3 Dev Module" | 115200 baudios | librerias: ESP32Servo y WiFi
 *
 * Enlace: Wi-Fi (red "eArm_Gestos", TCP 8080, IP 192.168.4.1) y USB serie a la vez, mismos comandos.
 * Cada mensaje termina en salto de linea (\n).
 *
 * Maquina de estados:
 *   IDLE   en espera; acepta START, RESET, MONTAJE, TRABAJO y HOME; rechaza movimientos.
 *   ACTIVE ejecuta movimientos; si pasan TIMEOUT_MS sin mensajes (watchdog) pasa a ERROR.
 *   SAFE   paro de emergencia: congela los servos y rechaza movimientos.
 *   ERROR  fallo de enlace: el brazo baja solo a la pose de REPOSO, suena el zumbador y relaja los servos.
 *   De SAFE y ERROR solo se sale con RESET (a IDLE) y luego START.
 *
 * Comandos (Python -> ESP32): PING, START, STOP, SAFE, RESET, A,90 (A..D + angulo, solo en ACTIVE),
 *   MONTAJE, TRABAJO, HOME, REL (suelta los servos), STATUS.
 * Respuestas: PONG, ACK:<estado>, OK ..., ERR:<estado>, STATE:<estado>, DATA:a,b,c,d (cada 250 ms),
 *   EVT:REPOSO, INFO:reset=<motivo> uptime_ms=<ms> (al conectarse un cliente).
 *
 * Servos: A=GPIO4 base | B=GPIO5 brazo inferior | C=GPIO6 antebrazo | D=GPIO7 pinza. Zumbador: GPIO9.
 * Sentidos: A+ izquierda | B+ y C+ bajan | D+ abre.
 */
#include <WiFi.h>
#include <ESP32Servo.h>
#ifdef ARDUINO_ARCH_ESP32
#include <esp_system.h>
#endif

// ---- configuracion
const char* WIFI_SSID = "eArm_Gestos";
const char* WIFI_PASS = "brazo1234";          // minimo 8 caracteres
const uint16_t TCP_PORT = 8080;

const unsigned long TIMEOUT_MS = 1500;        // watchdog: sin mensajes en ACTIVE -> ERROR
const unsigned long CLIENTE_DEBOUNCE_MS = 2000; // el cliente Wi-Fi solo se da por caido si connected() falla este tiempo seguido
const unsigned long DATA_MS = 250;            // periodo de la telemetria DATA:

const int N = 4;
const char NAMES[N] = {'A', 'B', 'C', 'D'};
const int PINS[N]   = {4, 5, 6, 7};
const int HOME[N]   = {90, 120, 60, 90};      // posicion de trabajo del fabricante
// Limites segun calibracion.json (usar_min / usar_max)
const int LO[N]     = {14, 54, 8, 8};
const int HI[N]     = {172, 172, 172, 98};

// Pose de REPOSO del fail-safe (ERROR): brazo bajo, poco par en los servos. AJUSTALA mirando tu brazo.
// A y B/C se mueven a esta pose; la pinza D conserva su angulo. Debe estar dentro de LO/HI.
const int REPOSO[N] = {90, 150, 100, 90};
const bool RELAJAR_AL_LLEGAR = true;          // true: 1 s despues de llegar se sueltan los servos (sin ruido ni consumo)
const unsigned long RELAJAR_MS = 1000;
const int STEP_REPOSO_MS = 20;                // ms por grado al bajar a reposo (mas lento que el uso normal)
const int PITIDOS_ERROR = 3;                  // pitidos de aviso en ERROR; luego silencio

const bool MONTAJE_AL_ARRANCAR = false;       // true: al encender los 4 servos van a 90 grados

const int BUZZER_PIN = 9;
const int BEEP_ON = LOW;
const int BEEP_OFF = HIGH;
const int STEP_MS = 5;                        // ms por grado (5 ms = ~200 grados/s)

// ---- variables
enum Estado { ST_IDLE, ST_ACTIVE, ST_SAFE, ST_ERROR };
const char* NOMBRE_ESTADO[] = {"IDLE", "ACTIVE", "SAFE", "ERROR"};
Estado estadoActual = ST_IDLE;
unsigned long ultimoMensajeMs = 0;            // millis() del ultimo mensaje recibido (watchdog)
unsigned long ultimoDataMs = 0;
unsigned long ultimoPitidoMs = 0;
bool pitidoActivo = false;
int pitidosRestantes = 0;
bool reposoLogrado = false;
bool relajado = false;
unsigned long reposoMs = 0;

Servo servos[N];
bool attached[N] = {false, false, false, false};
int current[N]   = {90, 120, 60, 90};
int target[N]    = {90, 120, 60, 90};
unsigned long lastStep = 0;

WiFiServer servidor(TCP_PORT);
WiFiClient cliente;
bool clienteActivo = false;
unsigned long clienteCaidoDesde = 0;
String bufSerial = "";
String bufTcp = "";

// ---- utilidades
// Motivo del ultimo arranque: 1 = encendido, 3 = software, 4 = fallo, 5 a 7 = watchdog, 9 = bajon de voltaje (brownout)
int motivoReinicio() {
#ifdef ARDUINO_ARCH_ESP32
  return (int)esp_reset_reason();
#else
  return 0;
#endif
}

void beep(int ms) {
  digitalWrite(BUZZER_PIN, BEEP_ON);
  delay(ms);
  digitalWrite(BUZZER_PIN, BEEP_OFF);
}

void responder(Print &out, const String &texto) {
  String linea = texto;
  linea += '\n';
  out.print(linea);                           // una sola escritura por mensaje
}

void emitir(const String &texto) {            // a USB y al cliente Wi-Fi
  responder(Serial, texto);
  if (clienteActivo) responder(cliente, texto);
}

String textoData() {
  return String("DATA:") + current[0] + "," + current[1] + "," + current[2] + "," + current[3];
}

int indexOf(char c) {
  c = toupper(c);
  for (int i = 0; i < N; i++) if (NAMES[i] == c) return i;
  return -1;
}

void attachServo(int i) {
  if (attached[i]) return;
  servos[i].setPeriodHertz(50);
  servos[i].attach(PINS[i], 500, 2400);
  current[i] = HOME[i];
  target[i] = HOME[i];
  servos[i].write(current[i]);
  attached[i] = true;
  delay(300);
}

void attachServoAt(int i, int angle) {
  if (!attached[i]) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(PINS[i], 500, 2400);
    attached[i] = true;
  }
  current[i] = angle;
  target[i] = angle;
  servos[i].write(angle);
}

void congelar() {                             // detiene el movimiento y SOSTIENE la posicion actual
  for (int i = 0; i < N; i++) target[i] = current[i];
}

// Fail-safe de ERROR: A, B y C bajan despacio a la pose de reposo; D conserva su angulo.
void bajarAReposo() {
  for (int i = 0; i < N; i++) target[i] = (i == 3) ? current[i] : constrain(REPOSO[i], LO[i], HI[i]);
  reposoLogrado = false;
  relajado = false;
  pitidosRestantes = PITIDOS_ERROR;
  ultimoPitidoMs = 0;
}

// ---- FSM
void cambiarEstado(Estado nuevo) {
  if (nuevo == estadoActual) return;
  estadoActual = nuevo;
  if (nuevo == ST_SAFE) congelar();
  if (nuevo == ST_ERROR) bajarAReposo();
  emitir(String("STATE:") + NOMBRE_ESTADO[nuevo]);
  if (nuevo == ST_SAFE) beep(120);
}

bool puedeConfigurar() {                      // MONTAJE / TRABAJO / HOME
  return estadoActual == ST_IDLE || estadoActual == ST_ACTIVE;
}

// ---- comandos
void handleLine(String line, Print &out) {
  line.trim();
  if (line.length() == 0) return;
  ultimoMensajeMs = millis();

  if (line == "PING") { responder(out, "PONG"); return; }

  if (line == "SAFE") { cambiarEstado(ST_SAFE); responder(out, "ACK:SAFE"); return; }

  if (line == "RESET") { cambiarEstado(ST_IDLE); responder(out, "ACK:IDLE"); return; }

  if (line == "START") {
    if (estadoActual == ST_IDLE || estadoActual == ST_ACTIVE) {
      cambiarEstado(ST_ACTIVE);
      responder(out, "ACK:ACTIVE");
    } else {
      responder(out, String("ERR:") + NOMBRE_ESTADO[estadoActual]);
    }
    return;
  }

  if (line == "STOP") {
    if (estadoActual == ST_ACTIVE) cambiarEstado(ST_IDLE);
    responder(out, String("ACK:") + NOMBRE_ESTADO[estadoActual]);
    return;
  }

  if (line == "STATUS") {
    responder(out, String("STATE:") + NOMBRE_ESTADO[estadoActual]);
    responder(out, textoData());
    return;
  }

  if (line == "REL") {                        // siempre permitido (apagado / emergencia)
    for (int i = 0; i < N; i++) {
      if (attached[i]) { servos[i].detach(); attached[i] = false; }
    }
    responder(out, "OK REL");
    return;
  }

  if (line == "MONTAJE" || line == "TRABAJO" || line == "HOME") {
    if (!puedeConfigurar()) { responder(out, String("ERR:") + NOMBRE_ESTADO[estadoActual]); return; }
    if (line == "MONTAJE") {
      for (int i = 0; i < N; i++) { attachServoAt(i, 90); delay(300); }
    } else if (line == "TRABAJO") {
      for (int i = 0; i < N; i++) {
        if (!attached[i]) { attachServoAt(i, current[i]); delay(300); }
        target[i] = HOME[i];
      }
    } else {
      for (int i = 0; i < N; i++) if (attached[i]) target[i] = HOME[i];
    }
    responder(out, String("OK ") + line);
    ultimoMensajeMs = millis();               // los delay() de arriba no cuentan como silencio
    return;
  }

  // Formato "A,90": solo se ejecuta en ACTIVE
  if (line.length() >= 3 && line.charAt(1) == ',' && isDigit(line.charAt(2))) {
    int i = indexOf(line.charAt(0));
    if (i >= 0) {
      if (estadoActual != ST_ACTIVE) { responder(out, String("ERR:") + NOMBRE_ESTADO[estadoActual]); return; }
      int angle = line.substring(2).toInt();
      int clamped = constrain(angle, LO[i], HI[i]);
      attachServo(i);
      target[i] = clamped;
      ultimoMensajeMs = millis();
      if (clamped != angle) responder(out, String("OK ") + NAMES[i] + " " + clamped + " (limitado, pediste " + angle + ")");
      else                  responder(out, String("OK ") + NAMES[i] + " " + clamped);
      return;
    }
  }

  responder(out, String("ERR:COMANDO ") + line);
}

void leerLineas(Stream &s, String &buf, Print &out) {
  while (s.available()) {
    char c = s.read();
    if (c == '\n') { handleLine(buf, out); buf = ""; }
    else if (c != '\r' && buf.length() < 80) buf += c;
  }
}

// ---- setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BEEP_OFF);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  delay(500);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
#ifdef ARDUINO_ARCH_ESP32
  WiFi.setTxPower(WIFI_POWER_8_5dBm);          // menos potencia de radio = picos de corriente menores (evita brownout)
#endif
  servidor.begin();
  servidor.setNoDelay(true);

  Serial.print("RESET_REASON:");
  Serial.print(motivoReinicio());
  Serial.print('\n');
  Serial.print("READY FINAL - Wi-Fi: ");
  Serial.print(WIFI_SSID);
  Serial.print(" IP ");
  Serial.print(WiFi.softAPIP());
  Serial.print(" puerto ");
  Serial.print(TCP_PORT);
  Serial.print('\n');
  responder(Serial, "STATE:IDLE");

  if (MONTAJE_AL_ARRANCAR) {
    beep(100);
    for (int i = 0; i < N; i++) { attachServoAt(i, 90); delay(300); }
    responder(Serial, "OK MONTAJE");
  }
  ultimoMensajeMs = millis();
}

void loop() {
  // 1) nuevo cliente Wi-Fi: si llega uno, reemplaza al anterior
  WiFiClient nuevo = servidor.available();
  if (nuevo) {
    cliente.stop();
    cliente = nuevo;
    cliente.setNoDelay(true);
    bufTcp = "";
    clienteActivo = true;
    clienteCaidoDesde = 0;
    responder(cliente, "READY");
    responder(cliente, String("INFO:reset=") + String(motivoReinicio()) + " uptime_ms=" + String((int)millis()));
    responder(cliente, String("STATE:") + NOMBRE_ESTADO[estadoActual]);
  }
  // el cliente solo se descarta si connected() falla de forma continua (evita falsas caidas)
  if (clienteActivo) {
    if (!cliente.connected()) {
      if (clienteCaidoDesde == 0) clienteCaidoDesde = millis();
      else if (millis() - clienteCaidoDesde > CLIENTE_DEBOUNCE_MS) { clienteActivo = false; cliente.stop(); }
    } else {
      clienteCaidoDesde = 0;
    }
  }

  // 2) leer comandos de USB y de Wi-Fi
  leerLineas(Serial, bufSerial, Serial);
  if (clienteActivo) leerLineas(cliente, bufTcp, cliente);

  // 3) watchdog: en ACTIVE, si no llega nada en TIMEOUT_MS -> ERROR (servos congelados)
  if (estadoActual == ST_ACTIVE && millis() - ultimoMensajeMs > TIMEOUT_MS) {
    cambiarEstado(ST_ERROR);
  }

  // 4) telemetria periodica
  if (millis() - ultimoDataMs >= DATA_MS) {
    ultimoDataMs = millis();
    emitir(textoData());
  }

  // 5) movimiento gradual hacia el objetivo (congelado en SAFE; en ERROR baja despacio a reposo)
  unsigned long pasoMs = (estadoActual == ST_ERROR) ? STEP_REPOSO_MS : STEP_MS;
  if (millis() - lastStep >= pasoMs) {
    lastStep = millis();
    for (int i = 0; i < N; i++) {
      if (attached[i] && current[i] != target[i]) {
        current[i] += (target[i] > current[i]) ? 1 : -1;
        servos[i].write(current[i]);
      }
    }
  }

  // 6) ERROR: avisar al llegar a reposo y, opcionalmente, relajar los servos
  if (estadoActual == ST_ERROR && !reposoLogrado) {
    bool llego = true;
    for (int i = 0; i < N; i++) if (attached[i] && current[i] != target[i]) llego = false;
    if (llego) { reposoLogrado = true; reposoMs = millis(); emitir("EVT:REPOSO"); }
  }
  if (estadoActual == ST_ERROR && reposoLogrado && RELAJAR_AL_LLEGAR && !relajado && millis() - reposoMs > RELAJAR_MS) {
    for (int i = 0; i < N; i++) if (attached[i]) { servos[i].detach(); attached[i] = false; }
    relajado = true;
  }

  // 7) alarma en ERROR: PITIDOS_ERROR pitidos y luego silencio (sin bloquear)
  unsigned long t = millis();
  if (estadoActual == ST_ERROR && pitidosRestantes > 0) {
    if (!pitidoActivo && t - ultimoPitidoMs > 1000) {
      ultimoPitidoMs = t; digitalWrite(BUZZER_PIN, BEEP_ON); pitidoActivo = true; pitidosRestantes--;
    }
  }
  if (pitidoActivo && (t - ultimoPitidoMs > 100 || estadoActual != ST_ERROR)) {
    digitalWrite(BUZZER_PIN, BEEP_OFF);
    pitidoActivo = false;
  }
}
