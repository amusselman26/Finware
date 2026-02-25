// Feather9x_SerialLoRa_Bridge
// - Continuously reads lines from Serial and sends them over LoRa as TEXT packets
// - Receives LoRa packets and prints them to Serial
// - Also decodes TelemetryLLA packets if they show up
//
// Usage:
// - Open Serial Monitor / Python script at 115200 baud
// - Send newline-terminated strings (e.g., "hello\n")
// - Feather transmits each line immediately over LoRa
//
// Notes:
// - LoRa payload max is RH_RF95_MAX_MESSAGE_LEN (~251 bytes). We keep margin.
// - This code uses a 1-byte type header: [type][payload...]
//   So the other side should also expect that (recommended).
// - If you need to talk to a receiver that sends/expects RAW strings without the type byte,
//   tell me and I’ll give you the “raw mode” variant.

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   4
#define RFM95_INT  3
#define RFM95_RST  2
#define RF95_FREQ  915.0

// -------- Packet type header (recommended) --------
enum PacketType : uint8_t {
  PKT_LLA  = 0x01,   // payload = TelemetryLLA
  PKT_TEXT = 0x02,   // payload = string bytes (no null terminator required)
  PKT_ACK  = 0x03
};

// Optional binary telemetry struct (if you still want to receive it)
struct __attribute__((packed)) TelemetryLLA {
  int32_t  lat_e7;  // degrees * 1e7
  int32_t  lon_e7;  // degrees * 1e7
  float    alt_m;   // meters
  uint32_t t_ms;    // TX timestamp (ms)
};

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// -------- Serial line buffer --------
static char lineBuf[220];     // keep < ~250 total packet. (1-byte header + payload)
static uint16_t lineLen = 0;

// -------- Helpers --------
void hardResetLoRa() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
}

bool isLikelyLLA(const TelemetryLLA& pkt) {
  const double lat = pkt.lat_e7 / 1e7;
  const double lon = pkt.lon_e7 / 1e7;
  const float alt  = pkt.alt_m;

  const bool lat_ok = (lat >= -90.0 && lat <= 90.0);
  const bool lon_ok = (lon >= -180.0 && lon <= 180.0);
  const bool alt_ok = (alt > -1000.0f && alt < 100000.0f); // -1 km to 100 km
  return lat_ok && lon_ok && alt_ok;
}

void sendTextLoRa(const uint8_t* data, uint8_t n) {
  // Packet: [PKT_TEXT][payload...]
  uint8_t out[RH_RF95_MAX_MESSAGE_LEN];
  if (n > (RH_RF95_MAX_MESSAGE_LEN - 1)) n = (RH_RF95_MAX_MESSAGE_LEN - 1);

  out[0] = PKT_TEXT;
  memcpy(&out[1], data, n);
  Serial.println("Sending Packet");

  rf95.send(out, 1 + n);
  rf95.waitPacketSent();
  Serial.println("Packet Sent");
}

void sendAckLoRa() {
  uint8_t out[1] = { PKT_ACK };
  rf95.send(out, sizeof(out));
  rf95.waitPacketSent();
}

void printAsSafeAscii(const uint8_t* payload, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    char c = (char)payload[i];
    Serial.print((c >= 32 && c <= 126) ? c : '.');
  }
}

void handleSerialToLoRa() {
  // Non-blocking: accumulate until newline, then send immediately
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;  // ignore CR

    // Backspace support (optional)
    if (c == '\b' || c == 127) {
      if (lineLen > 0) lineLen--;
      continue;
    }

    if (c == '\n') {
      // End of line -> send if non-empty
      if (lineLen > 0) {
        sendTextLoRa((uint8_t*)lineBuf, (uint8_t)lineLen);

        // Optional local echo / debug
        Serial.print("TX> ");
        Serial.write((uint8_t*)lineBuf, lineLen);
        Serial.println();

        lineLen = 0;
      }
      continue;
    }

    // Regular char
    if (lineLen < (sizeof(lineBuf) - 1)) {
      lineBuf[lineLen++] = c;
    } else {
      // Buffer full -> send what we have (chunk) and reset.
      // (This keeps “continuous” data from stalling.)
      sendTextLoRa((uint8_t*)lineBuf, (uint8_t)lineLen);

      Serial.print("TX(CHUNK)> ");
      Serial.write((uint8_t*)lineBuf, lineLen);
      Serial.println();

      lineLen = 0;
      // Add current char as start of next chunk if not newline
      lineBuf[lineLen++] = c;
    }
  }
}

void handleLoRaToSerial() {
  if (!rf95.available()) return;

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (!rf95.recv(buf, &len)) {
    // CRC/header error
    return;
  }

  if (len < 1) return;

  const int rssi = rf95.lastRssi();

  // 1) Typed packets
  const uint8_t ptype = buf[0];
  const uint8_t* payload = &buf[1];
  const uint8_t plen = len - 1;

  if (ptype == PKT_TEXT) {
    // Print payload as a line so computer can parse easily
    Serial.print("RX< ");
    Serial.write(payload, plen);
    Serial.print(" | RSSI=");
    Serial.println(rssi);

    // Optional ACK
    // sendAckLoRa();
    return;
  }

  if (ptype == PKT_LLA) {
    if (plen == sizeof(TelemetryLLA)) {
      TelemetryLLA pkt;
      memcpy(&pkt, payload, sizeof(pkt));
      if (isLikelyLLA(pkt)) {
        const double lat = pkt.lat_e7 / 1e7;
        const double lon = pkt.lon_e7 / 1e7;
        const float  alt = pkt.alt_m;

        Serial.print("RX LLA< ");
        Serial.print(lat, 7); Serial.print(", ");
        Serial.print(lon, 7); Serial.print(", ");
        Serial.print(alt, 2); Serial.print(" m");
        Serial.print(" | t_ms=");
        Serial.print(pkt.t_ms);
        Serial.print(" | RSSI=");
        Serial.println(rssi);
        return;
      }
    }
    Serial.print("RX LLA< (bad) plen=");
    Serial.print(plen);
    Serial.print(" | RSSI=");
    Serial.println(rssi);
    return;
  }

  if (ptype == PKT_ACK) {
    Serial.print("RX ACK< RSSI=");
    Serial.println(rssi);
    return;
  }

  // 2) Fallback: maybe it's RAW TelemetryLLA (no type byte)
  if (len == sizeof(TelemetryLLA)) {
    TelemetryLLA pkt;
    memcpy(&pkt, buf, sizeof(pkt));
    if (isLikelyLLA(pkt)) {
      const double lat = pkt.lat_e7 / 1e7;
      const double lon = pkt.lon_e7 / 1e7;
      const float  alt = pkt.alt_m;

      Serial.print("RX RAW LLA< ");
      Serial.print(lat, 7); Serial.print(", ");
      Serial.print(lon, 7); Serial.print(", ");
      Serial.print(alt, 2); Serial.print(" m");
      Serial.print(" | t_ms=");
      Serial.print(pkt.t_ms);
      Serial.print(" | RSSI=");
      Serial.println(rssi);
      return;
    }
  }

  // 3) Fallback: unstructured / raw bytes — print safe ASCII
  Serial.print("RX RAW< ");
  printAsSafeAscii(buf, len);
  Serial.print(" | len=");
  Serial.print(len);
  Serial.print(" | RSSI=");
  Serial.println(rssi);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Serial <-> LoRa Bridge (TEXT + optional LLA)");

  hardResetLoRa();

  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1) delay(1000);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1) delay(1000);
  }

  // Match settings on both ends if you change them:
  // rf95.setSignalBandwidth(125000);
  // rf95.setSpreadingFactor(7);
  // rf95.setCodingRate4(5);

  rf95.setTxPower(23, false);

  Serial.print("Ready @ "); Serial.print(RF95_FREQ); Serial.println(" MHz");
  Serial.println("Send newline-terminated lines over Serial to transmit over LoRa.");
}

void loop() {
  // Do both directions continuously
  handleSerialToLoRa();
  handleLoRaToSerial();
}
