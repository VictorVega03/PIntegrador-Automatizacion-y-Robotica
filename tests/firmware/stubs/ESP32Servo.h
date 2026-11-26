#pragma once
#include "Arduino.h"
extern std::vector<std::pair<int,int>> g_writes;
class Servo {
 public:
  int pin = -1; bool att = false; int ang = -1;
  void setPeriodHertz(int) {}
  void attach(int p, int, int) { pin = p; att = true; }
  void detach() { att = false; }
  void write(int a) { ang = a; g_writes.push_back({pin, a}); }
};
struct ESP32PWM { static void allocateTimer(int) {} };
