// SD card write latency benchmark
// Measures average time to write 160 bytes to the SD card.

#include <STM32SD.h>

// If SD card slot has no detect pin then define it as SD_DETECT_NONE
// to ignore it. One other option is to call 'SD.begin()' without parameter.
#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN SD_DETECT_NONE
#endif

// Configuration
static const size_t WRITE_SIZE      = 160;   // bytes per write
static const size_t NUM_ITERATIONS  = 1000;  // number of writes to average
static const char *TEST_FILENAME    = "latency.bin";

File myFile;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  Serial.println("SD write latency test");
  Serial.print("Write size (bytes): ");
  Serial.println(WRITE_SIZE);
  Serial.print("Iterations: ");
  Serial.println(NUM_ITERATIONS);

  Serial.print("Initializing SD card...");
  while (!SD.begin()) {
    delay(10);
  }
  Serial.println(" done.");

  // Remove any existing test file so we start from a clean file.
  SD.remove(TEST_FILENAME);

  myFile = SD.open(TEST_FILENAME, FILE_WRITE);
  if (!myFile) {
    Serial.println("Failed to open test file for writing.");
    return;
  }

  // Warm-up: simple text write to verify that writing works at all.
  size_t warmupWritten = myFile.println("warmup");
  myFile.flush();
  Serial.print("Warmup write bytes: ");
  Serial.println(warmupWritten);

  // Prepare a fixed 160-byte buffer.
  uint8_t buffer[WRITE_SIZE];
  for (size_t i = 0; i < WRITE_SIZE; ++i) {
    buffer[i] = (uint8_t)(i & 0xFF);
  }

  unsigned long totalMicros = 0;
  unsigned long minMicros = 0xFFFFFFFFUL;
  unsigned long maxMicros = 0;

  Serial.println("Starting binary 160-byte writes...");
  myFile.flush();

  for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
    unsigned long start = micros();
    size_t written = myFile.write(buffer, WRITE_SIZE);
    // Optionally include flush cost; enable if desired.
    myFile.flush();
    unsigned long elapsed = micros() - start;

    if (written != WRITE_SIZE) {
      Serial.print("Short write at iteration ");
      Serial.print(i);
      Serial.print(", wrote ");
      Serial.print(written);
      Serial.println(" bytes");
    }

    totalMicros += elapsed;
    if (elapsed < minMicros) {
      minMicros = elapsed;
    }
    if (elapsed > maxMicros) {
      maxMicros = elapsed;
    }

    // Optional: print progress every 100 writes
    if ((i + 1) % 100 == 0) {
      Serial.print("Completed ");
      Serial.print(i + 1);
      Serial.println(" writes...");
    }
  }

  myFile.close();

  float avgMicros = (float)totalMicros / (float)NUM_ITERATIONS;

  Serial.println();
  Serial.println("===== SD 160-byte Write Latency Results =====");
  Serial.print("Total writes: ");
  Serial.println(NUM_ITERATIONS);
  Serial.print("Bytes per write: ");
  Serial.println(WRITE_SIZE);
  Serial.print("Average time (us): ");
  Serial.println(avgMicros, 3);
  Serial.print("Min time (us): ");
  Serial.println(minMicros);
  Serial.print("Max time (us): ");
  Serial.println(maxMicros);

  if (!SD.end()) {
    Serial.println("Failed to properly end the SD.");
  }

  Serial.println("Test complete.");
}

void loop() {
  // Nothing to do; test runs once in setup().
}