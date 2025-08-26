## 2025-08-15
Goal: Bring up LPS22HB and BNO085 on Feather F405 over I2C.
Issue: I2C scan showed no devices; SCL=0V.
Fix: Resolder SCL header on LPS22HB; scanner found Ox5C.
Next: SD SPI test

## 2025-08-15 TO-DO
1. Log to feather SDIO
2. Add simulink model to github
3. Pressure to altitude conversion from barometer
4. Test barometer and BNO085 on I2C bus
5. Order GNSS
6. Test LoRa
7. Create I/O table

## 2025-08-21
Goal: Integrate BNO085 with LPS22
Next: Develop testing tools, test drift of IMU and noise of LPS22

## 2025-08-21
Goal: Add initial atmospheric pressure calibration to lps22 example
Next: Test BNO and LPS on SPI - SPI should be favored over I2C

## 2025-08-25
Goal: create system architecture and BNO085 driver
Issue: Arduino linker not recognizing .cpp files in external directories
Fix: Main ino file should be placed in sketch folder with same name. header and .cpp files should be in subdirectories. This will require changing the src directory. 
Next: Develop LPS22 driver, develop testing tools, test drift and noise of sensors. Implement LoRa module.
