#pragma once
#include <string>
#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <cctype>
#include <deque>
#include <vector>
#define LOW 0
#define HIGH 1
#define OUTPUT 1
template <class T, class U, class V> T constrain(T x, U lo, V hi) { return x < (T)lo ? (T)lo : (x > (T)hi ? (T)hi : x); }
inline bool isDigit(char c) { return c >= '0' && c <= '9'; }
extern unsigned long g_ms;
inline unsigned long millis() { return g_ms; }
inline void delay(unsigned long ms) { g_ms += ms; }
extern std::vector<std::pair<int,int>> g_pin_log;
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { g_pin_log.push_back({p, v}); }
class String : public std::string {
 public:
  String() {}
  String(const char* s) : std::string(s) {}
  String(const std::string& s) : std::string(s) {}
  String(int v) : std::string(std::to_string(v)) {}
  String(unsigned v) : std::string(std::to_string(v)) {}
  void trim() { size_t a = find_first_not_of(" \t\r\n"); if (a == npos) { clear(); return; } size_t b = find_last_not_of(" \t\r\n"); *this = String(substr(a, b - a + 1)); }
  size_t length() const { return size(); }
  char charAt(size_t i) const { return (*this)[i]; }
  String substring(size_t a) const { return String(substr(a)); }
  int toInt() const { return atoi(c_str()); }
  String operator+(const String& o) const { return String(std::string(*this) + std::string(o)); }
  String operator+(const char* o) const { return String(std::string(*this) + o); }
  String operator+(char c) const { return String(std::string(*this) + c); }
  String operator+(int v) const { return String(std::string(*this) + std::to_string(v)); }
  String& operator+=(char c) { append(1, c); return *this; }
  bool operator==(const char* o) const { return std::string(*this) == o; }
};
class Print {
 public:
  virtual ~Print() {}
  virtual void write1(char c) = 0;
  void print(const String& s) { for (char c : s) write1(c); }
  void print(const char* s) { while (*s) write1(*s++); }
  void print(char c) { write1(c); }
  void print(int v) { print(String(v)); }
  void print(unsigned v) { print(String(v)); }
};
class Stream : public Print {
 public:
  std::deque<char> in; std::string out;
  void write1(char c) override { out.push_back(c); }
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return c; }
  void feed(const std::string& s) { for (char c : s) in.push_back(c); }
};
class HardwareSerial : public Stream { public: void begin(int) {} };
extern HardwareSerial Serial;
