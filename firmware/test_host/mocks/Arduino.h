// Minimal stand-in for the Arduino-ESP32 core, used only by `make sketch-check`
// to type-check src/*.cpp on a PC.  Nothing here runs.
#pragma once
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define ESP_ARDUINO_VERSION_MAJOR 2
#define IRAM_ATTR
#define PROGMEM
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
enum adc_attenuation_t { ADC_0db, ADC_2_5db, ADC_6db, ADC_11db };

void pinMode(int, int);
void digitalWrite(int, int);
int digitalRead(int);
int analogRead(int);
uint32_t analogReadMilliVolts(int);
void analogReadResolution(int);
void analogSetPinAttenuation(int, adc_attenuation_t);
uint32_t micros();
uint32_t millis();
void delay(uint32_t);
void delayMicroseconds(uint32_t);
void attachInterrupt(int, void (*)(), int);
int digitalPinToInterrupt(int);
void noInterrupts();
void interrupts();
double ledcSetup(uint8_t, double, uint8_t);
void ledcAttachPin(uint8_t, uint8_t);
void ledcWrite(uint8_t, uint32_t);

class String {
 public:
  String(const char* s = "");
  const char* c_str() const;
  size_t length() const;
};

class HardwareSerial {
 public:
  void begin(unsigned long);
  int available();
  int read();
  size_t print(const char*);
  size_t println(const char* = "");
  size_t printf(const char*, ...) __attribute__((format(printf, 2, 3)));
};
extern HardwareSerial Serial;
