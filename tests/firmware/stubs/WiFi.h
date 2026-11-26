#pragma once
#include "Arduino.h"
#define WIFI_AP 1
struct IPAddressStub {};
inline Print& operator<<(Print& p, IPAddressStub) { return p; }
class WiFiClient : public Stream {
 public:
  bool conectado = false;
  explicit operator bool() const { return conectado; }
  bool connected() { return conectado; }
  void stop() { conectado = false; }
  void setNoDelay(bool) {}
};
class WiFiServer {
 public:
  WiFiClient pendiente; bool hay = false;
  WiFiServer(uint16_t) {}
  void begin() {}
  void setNoDelay(bool) {}
  WiFiClient available() { if (hay) { hay = false; WiFiClient c = pendiente; c.conectado = true; return c; } return WiFiClient(); }
};
struct WiFiClass { void mode(int) {} void softAP(const char*, const char*) {} int softAPIP() { return 0; } };
extern WiFiClass WiFi;
