#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---------------- Servo Settings ----------------
#define SERVO_FREQ 50      // 50 Hz for analog servos
#define MIN_US 1000        // safe lower bound
#define MAX_US 2000        // safe upper bound
#define START_ZERO_US 1500 // initial "0" guess (center)

// PCA9685 runs at 4096 counts over 20ms at 50Hz
// 20ms = 20000us
// 1 count = 20000 / 4096 = ~4.88us

uint16_t usToTicks(uint16_t us) {
  return (uint16_t)((us * 4096.0) / 20000.0);
}

// ---------------- Calibration State ----------------
uint8_t selectedServo = 0;
uint16_t servoPulseWidth[16];

void setServo(uint8_t ch, uint16_t pulse_us) {
  if (pulse_us < MIN_US) pulse_us = MIN_US;
  if (pulse_us > MAX_US) pulse_us = MAX_US;

  servoPulseWidth[ch] = pulse_us;
  uint16_t ticks = usToTicks(pulse_us);
  pwm.setPWM(ch, 0, ticks);
}

void resetServo(uint8_t ch) {
  setServo(ch, START_ZERO_US);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // Initialize all servos to START_ZERO_US
  for (int i = 0; i < 16; i++) {
    servoPulseWidth[i] = START_ZERO_US;
    setServo(i, START_ZERO_US);
  }

  Serial.println("==== SERVO CALIBRATION MODE ====");
  Serial.println("Commands:");
  Serial.println("s X  -> select servo (0-15)");
  Serial.println("+    -> increase 5us");
  Serial.println("-    -> decrease 5us");
  Serial.println("z    -> reset to start zero");
  Serial.println("p    -> print current pulse");
  Serial.println("================================");
}

void loop() {

  if (Serial.available()) {

    char cmd = Serial.read();

    if (cmd == 's') {
      while (!Serial.available());
      selectedServo = Serial.parseInt();
      if (selectedServo > 15) selectedServo = 15;

      Serial.print("Selected Servo: ");
      Serial.println(selectedServo);
    }

    else if (cmd == '+') {
      setServo(selectedServo, servoPulseWidth[selectedServo] + 5);
      Serial.print("Servo ");
      Serial.print(selectedServo);
      Serial.print(" = ");
      Serial.print(servoPulseWidth[selectedServo]);
      Serial.println(" us");
    }

    else if (cmd == '-') {
      setServo(selectedServo, servoPulseWidth[selectedServo] - 5);
      Serial.print("Servo ");
      Serial.print(selectedServo);
      Serial.print(" = ");
      Serial.print(servoPulseWidth[selectedServo]);
      Serial.println(" us");
    }

    else if (cmd == 'z') {
      resetServo(selectedServo);
      Serial.print("Servo ");
      Serial.print(selectedServo);
      Serial.println(" reset to start zero");
    }

    else if (cmd == 'p') {
      Serial.print("Servo ");
      Serial.print(selectedServo);
      Serial.print(" pulse width: ");
      Serial.print(servoPulseWidth[selectedServo]);
      Serial.println(" us");
    }
  }
}
