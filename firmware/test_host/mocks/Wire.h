#pragma once
#include <stdint.h>
class TwoWire {
 public:
  bool begin(int sda, int scl);
  void setClock(uint32_t);
  void beginTransmission(uint8_t);
  size_t write(uint8_t);
  uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(int addr, int n);
  int read();
  int available();
};
extern TwoWire Wire;
