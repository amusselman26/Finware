#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

// Packet type header (must match ground receiver)
enum class LoRaPacketType : uint8_t {
  LLA  = 0x01,  // TelemetryLLA
  TEXT = 0x02,  // ASCII text / commands
  ACK  = 0x03
};

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

  // ---- Low-level raw send (no type header) ----
  bool sendRaw(const uint8_t* data, uint8_t len) {
    if (len == 0) return false;
    _rf95.send(data, len);
    _rf95.waitPacketSent();
    return true;
  }

  // Backwards-compatible alias (still available if needed)
  bool sendMessage(const uint8_t* data, uint8_t len) {
    return sendRaw(data, len);
  }

  // ---- Typed send helpers ----

  bool sendTyped(LoRaPacketType type, const uint8_t* payload, uint8_t len) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    if (len > (uint8_t)(sizeof(buf) - 1)) {
      len = (uint8_t)(sizeof(buf) - 1);
    }
    buf[0] = static_cast<uint8_t>(type);
    if (len > 0 && payload) {
      memcpy(&buf[1], payload, len);
    }
    return sendRaw(buf, (uint8_t)(len + 1));
  }

  bool sendText(const String& msg) {
    const uint8_t* data = (const uint8_t*)msg.c_str();
    uint8_t len = (uint8_t)msg.length(); // no null terminator on the air
    return sendTyped(LoRaPacketType::TEXT, data, len);
  }

  bool sendAck() {
    uint8_t dummy = 0; // no payload
    return sendTyped(LoRaPacketType::ACK, &dummy, 0);
  }

  // ---- Receive helpers ----

  // Typed receive: returns true only if a packet with a header byte is received.
  //  - outType: packet type
  //  - payload: pointer to payload buffer (without type header)
  //  - lenInOut: in = buffer size, out = payload length
  bool receiveTyped(LoRaPacketType& outType, uint8_t* payload, uint8_t& lenInOut,
                    uint16_t timeout_ms = 1000) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (!_rf95.waitAvailableTimeout(timeout_ms)) {
      return false;
    }

    if (!_rf95.recv(buf, &len)) {
      return false;
    }

    if (len < 1) {
      return false;
    }

    _lastRssi = _rf95.lastRssi();

    outType = static_cast<LoRaPacketType>(buf[0]);
    uint8_t payloadLen = (uint8_t)(len - 1);
    if (payload && lenInOut > 0 && payloadLen > 0) {
      if (payloadLen > lenInOut) {
        payloadLen = lenInOut;
      }
      memcpy(payload, &buf[1], payloadLen);
    }
    lenInOut = payloadLen;
    return true;
  }

  // Convenience wrapper for TEXT packets only.
  // Returns true if a TEXT packet was received and decoded into outMsg.
  bool receiveText(String& outMsg, uint16_t timeout_ms = 1000) {
    LoRaPacketType type;
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (!receiveTyped(type, buf, len, timeout_ms)) {
      return false;
    }

    if (type != LoRaPacketType::TEXT) {
      // Not text; caller can use receiveTyped() directly for other types.
      return false;
    }

    // Interpret payload as bytes -> String (no required null terminator)
    outMsg = String();
    for (uint8_t i = 0; i < len; ++i) {
      outMsg += (char)buf[i];
    }
    return true;
  }

  // Legacy API: kept for compatibility, now behaves like receiveText().
  // Only returns true when a TEXT packet is received.
  bool receiveMessage(String& outMsg, uint16_t timeout_ms = 1000) {
    return receiveText(outMsg, timeout_ms);
  }

  int16_t lastRssi() const { return _lastRssi; }

private:
  RH_RF95 _rf95;
  uint8_t _rst;
  float _freq;
  int16_t _lastRssi = 0;
};
