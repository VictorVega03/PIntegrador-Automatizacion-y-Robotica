#include "Arduino.h"
unsigned long g_ms = 1000;
std::vector<std::pair<int,int>> g_pin_log, g_writes;
HardwareSerial Serial;
#include "WiFi.h"
WiFiClass WiFi;
#include "../../firmware/PIntegrador_Brazo/PIntegrador_Brazo.ino"
#include <cassert>
#include <iostream>

static void correr(unsigned ms, unsigned paso = 5) { for (unsigned t = 0; t < ms; t += paso) { g_ms += paso; loop(); } }
static std::string tomar() { std::string s = Serial.out; Serial.out.clear(); return s; }
static bool tiene(const std::string& s, const std::string& x) { return s.find(x) != std::string::npos; }
static bool latido = true;
static std::string cmd(const std::string& c, unsigned ms = 20) {
  Serial.feed(c + "\n");
  for (unsigned t = 0; t < ms; t += 5) { if (latido && t % 200 == 0 && t > 0) Serial.feed("PING\n"); correr(5); }
  return tomar();
}
#define CHECK(c) do { if (!(c)) { std::cerr << "FALLO linea " << __LINE__ << ": " #c "\n"; return 1; } } while (0)

int main() {
  setup(); correr(50); tomar();
  CHECK(estadoActual == ST_IDLE);

  CHECK(tiene(cmd("A,100"), "ERR:IDLE"));
  CHECK(tiene(cmd("PING"), "PONG"));

  cmd("RESET"); CHECK(tiene(cmd("START"), "ACK:ACTIVE")); CHECK(estadoActual == ST_ACTIVE);
  CHECK(tiene(cmd("TRABAJO", 20), "OK TRABAJO"));
  std::string r = cmd("A,100", 400); CHECK(tiene(r, "OK A 100")); CHECK(current[0] == 100);
  r = cmd("A,200", 1500); CHECK(tiene(r, "OK A 172 (limitado, pediste 200)")); CHECK(current[0] == 172);
  r = cmd("D,0", 1500); CHECK(tiene(r, "OK D 8")); CHECK(current[3] == 8);
  CHECK(tiene(cmd("X,5"), "ERR:COMANDO"));
  r = tomar(); correr(300); r = tomar(); CHECK(tiene(r, "DATA:172,"));

  cmd("A,90", 20);
  CHECK(tiene(cmd("SAFE"), "ACK:SAFE")); CHECK(estadoActual == ST_SAFE);
  int a_congelado = current[0];
  r = cmd("A,140", 800); CHECK(tiene(r, "ERR:SAFE")); CHECK(current[0] == a_congelado); CHECK(target[0] == a_congelado);
  CHECK(tiene(cmd("START"), "ERR:SAFE"));
  CHECK(tiene(cmd("TRABAJO"), "ERR:SAFE"));
  correr(3000); CHECK(estadoActual == ST_SAFE);
  cmd("RESET"); CHECK(estadoActual == ST_IDLE); cmd("START"); CHECK(estadoActual == ST_ACTIVE);

  cmd("A,120", 5); latido = false;
  correr(300); tomar();
  correr(TIMEOUT_MS - 400); CHECK(estadoActual == ST_ACTIVE);
  g_pin_log.clear(); correr(600); CHECK(estadoActual == ST_ERROR);
  r = tomar(); CHECK(tiene(r, "STATE:ERROR"));
  CHECK(target[0] == 90 && target[1] == 150 && target[2] == 100);     // pose de reposo
  int d_antes = current[3]; CHECK(target[3] == d_antes);               // la pinza conserva su angulo
  g_writes.clear();
  correr(4200);
  int pitidos = 0; for (auto& p : g_pin_log) if (p.first == 9 && p.second == LOW) pitidos++;
  CHECK(pitidos == 3);                                                // 3 pitidos y luego silencio
  CHECK(current[0] == 90 && current[1] == 150 && current[2] == 100 && current[3] == d_antes);
  r = tomar(); CHECK(tiene(r, "EVT:REPOSO"));
  CHECK(!attached[0] && !attached[1] && !attached[2] && !attached[3]); // servos relajados
  // bajada gradual: nunca mas de 1 grado entre escrituras consecutivas del mismo servo
  { int ult[4] = {-1, -1, -1, -1}; for (auto& w : g_writes) { int i = w.first == 4 ? 0 : w.first == 5 ? 1 : w.first == 6 ? 2 : 3; if (ult[i] >= 0) CHECK(abs(w.second - ult[i]) <= 1); ult[i] = w.second; } }
  r = cmd("A,100", 300); latido = true; CHECK(tiene(r, "ERR:ERROR")); CHECK(estadoActual == ST_ERROR);
  cmd("RESET"); CHECK(estadoActual == ST_IDLE);
  cmd("START"); g_writes.clear(); cmd("TRABAJO", 30); CHECK(!g_writes.empty());
  CHECK(g_writes[0].second == 90 || g_writes[0].second == 150 || g_writes[0].second == 100);  // primera escritura = angulo de reposo, sin salto
  cmd("PING", 1500);

  cmd("START"); for (int k = 0; k < 40; k++) { cmd("PING", 100); } CHECK(estadoActual == ST_ACTIVE);

  cmd("REL"); CHECK(!attached[0]);
  Serial.feed("TRABAJO\nPING\nPING\n"); correr(30); std::cerr << "estado=" << NOMBRE_ESTADO[estadoActual] << " ms=" << g_ms << " ult=" << ultimoMensajeMs << " out=" << Serial.out.substr(0,120) << "\n"; CHECK(estadoActual == ST_ACTIVE);

  cmd("RESET");
  servidor.pendiente = WiFiClient(); servidor.hay = true; correr(20);
  CHECK(clienteActivo); CHECK(tiene(cliente.out, "READY")); cliente.out.clear();
  cliente.feed("START\n"); correr(20); CHECK(estadoActual == ST_ACTIVE); CHECK(tiene(cliente.out, "ACK:ACTIVE")); cliente.out.clear();
  cliente.feed("B,100\n"); correr(300); CHECK(tiene(cliente.out, "OK B 100"));
  for (int k = 0; k < 10; k++) { cliente.feed("PING\n"); correr(200); } CHECK(estadoActual == ST_ACTIVE);
  tomar(); cliente.out.clear();
  correr(TIMEOUT_MS + 300); CHECK(estadoActual == ST_ERROR); CHECK(tiene(cliente.out, "STATE:ERROR"));
  cliente.feed("RESET\n"); correr(20); CHECK(estadoActual == ST_IDLE);
  // antirrebote: connected() falso 500 ms no suelta al cliente; falso mas de 2 s si
  cliente.conectado = false; correr(500); CHECK(clienteActivo);
  cliente.conectado = true; correr(100); CHECK(clienteActivo);
  cliente.conectado = false; correr(2300); CHECK(!clienteActivo);
  // un cliente nuevo reemplaza al anterior
  servidor.pendiente = WiFiClient(); servidor.hay = true; correr(20); CHECK(clienteActivo); CHECK(tiene(cliente.out, "READY")); cliente.out.clear();

  cmd("START"); CHECK(tiene(cmd("STOP"), "ACK:IDLE")); r = cmd("STATUS"); CHECK(tiene(r, "STATE:IDLE") && tiene(r, "DATA:"));
  std::cout << "FIRMWARE FSM: TODAS LAS PRUEBAS OK\n";
  return 0;
}
