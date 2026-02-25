#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"

SDMMCBlockDevice sd;          // SDIO interface
FATFileSystem fs("sd");       // Mount point name

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=== SDIO SD Card Test ===");

  // Initialize SD card
  int err = sd.init();
  if (err) {
    Serial.print("SD init failed: ");
    Serial.println(err);
    while (1);
  }
  Serial.println("SD init OK");

  // Mount filesystem
  err = fs.mount(&sd);
  if (err) {
    Serial.print("Mount failed: ");
    Serial.println(err);
    while (1);
  }
  Serial.println("Filesystem mounted");

  // Write test file
  FILE *f = fopen("/sd/test.txt", "w");
  if (!f) {
    Serial.println("Failed to open file for writing");
    while (1);
  }

  fprintf(f, "Hello from STM32 SDIO!\n");
  fclose(f);
  Serial.println("Write OK");

  // Read test file
  f = fopen("/sd/test.txt", "r");
  if (!f) {
    Serial.println("Failed to open file for reading");
    while (1);
  }

  char buf[64];
  fgets(buf, sizeof(buf), f);
  fclose(f);

  Serial.print("Read back: ");
  Serial.println(buf);

  Serial.println("=== SDIO TEST PASSED ===");
}

void loop() {
  // Nothing here
}
