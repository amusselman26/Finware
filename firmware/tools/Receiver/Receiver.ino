// Feather9x_RX - binary LLA receiver
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   4
#define RFM95_INT  3
#define RFM95_RST  2
#define RF95_FREQ  915.0

// Must match the TX packet layout exactly
struct __attribute__((packed)) TelemetryLLA {
  int32_t  lat_e7;  // degrees * 1e7
  int32_t  lon_e7;  // degrees * 1e7
  float    alt_m;   // meters
  uint32_t t_ms;    // TX timestamp (ms)
};

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void hardResetLoRa() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("LoRa RX (binary LLA)");

  hardResetLoRa();

  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1) { delay(1000); }
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1) { delay(1000); }
  }
  // Match TX power/bw/sf if you changed them on TX; defaults are fine otherwise
  rf95.setTxPower(23, false);

  Serial.print("Listening at "); Serial.print(RF95_FREQ); Serial.println(" MHz");
}

void loop() {
  if (!rf95.available()) return;

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.recv(buf, &len)) {
    digitalWrite(LED_BUILTIN, HIGH);

    if (len == sizeof(TelemetryLLA)) {
      TelemetryLLA pkt;
      memcpy(&pkt, buf, sizeof(pkt));  // binary decode

      // Convert to human-friendly units
      const double lat = pkt.lat_e7 / 1e7;
      const double lon = pkt.lon_e7 / 1e7;
      const float  alt = pkt.alt_m;

      Serial.print("LLA: ");
      Serial.print(lat, 7); Serial.print(", ");
      Serial.print(lon, 7); Serial.print(", ");
      Serial.print(alt, 2); Serial.print(" m");
      Serial.print(" | t_ms: "); Serial.print(pkt.t_ms);
      Serial.print(" | RSSI: "); Serial.println(rf95.lastRssi());
    } else {
      // Length mismatch — maybe different packet version?
      Serial.print("Unexpected length: "); Serial.println(len);
    }

    digitalWrite(LED_BUILTIN, LOW);

    // No auto-reply — TX isn’t expecting one for telemetry
    // (If you want ACKs, add a small reply packet here.)
  } else {
    // CRC or header error
    // (Optional) Serial.println("Receive failed");
  }
}
