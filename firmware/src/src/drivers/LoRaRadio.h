#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

class LoRaRadio {
public:
  LoRaRadio(uint8_t csPin, uint8_t intPin, uint8_t rstPin, float freqMHz = 915.0)
    : _rf95(csPin, intPin), _rst(rstPin), _freq(freqMHz) {}

  bool begin(uint8_t txPower = 23) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(10);

    // Manual reset
    digitalWrite(_rst, LOW);
    delay(10);
    digitalWrite(_rst, HIGH);
    delay(10);

    if (!_rf95.init()) {
      Serial.println("LoRa init failed!");
      return false;
    }

    if (!_rf95.setFrequency(_freq)) {
      Serial.println("LoRa setFrequency failed!");
      return false;
    }

    _rf95.setTxPower(txPower, false);
    Serial.print("LoRa ready @ ");
    Serial.print(_freq, 1);
    Serial.println(" MHz");
    return true;
  }

  bool sendMessage(const uint8_t* data, uint8_t len) {
    if (len == 0) return false;
    _rf95.send(data, len);
    _rf95.waitPacketSent();
    return true;
  }

  bool sendMessage(const String& msg) {
    return sendMessage((const uint8_t*)msg.c_str(), msg.length() + 1);
  }

  // blocking receive with timeout (ms)
  bool receiveMessage(String& outMsg, uint16_t timeout_ms = 1000) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (_rf95.waitAvailableTimeout(timeout_ms)) {
      if (_rf95.recv(buf, &len)) {
        outMsg = String((char*)buf);
        _lastRssi = _rf95.lastRssi();
        return true;
      }
    }
    return false;
  }

  int16_t lastRssi() const { return _lastRssi; }

private:
  RH_RF95 _rf95;
  uint8_t _rst;
  float _freq;
  int16_t _lastRssi = 0;
};
