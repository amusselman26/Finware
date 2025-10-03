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

## 2025-08-26
Issue: Arduino linker can not find files in subdirectories of either the project or libraries folder.
Fix: Looking into usage of platformio. Currently cannot upload code through platformio
Next: Run feather test code on platformio

## 2025-08-27
Goal: Switch to platformio, test SparkFun GNSS over I2C.
Issue: platformio not reading serial messages on COM5.
Fix: Added build_flags = -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC.
Next: Add wrappers for LPS22 and GPS modules.

## 2025-09-04
Goal: Add LPS22 wrapper and test
Issue: Initial altitude in the -1000s, added if statement in example usage to recalibrate atm_pressure if alt less than -100 m.
Next: Add wrapper for GPS module

## 2025-09-10
GPS wrapper completed. Added adafruit documentation to repo. When trying to use STM32duino SD FAT, error in line 57 of SdFatFs.cpp prevented build. Changed this line locally to if (f_mount(NULL, (TCHAR const *)_SDPath, 0) == FR_OK). Pretty sure this will not port.

## 2025-09-30
Finalized wrappers for GPS, Baro, and IMU. Succesfully tested all three together. Working on RFM95 radio module and SD card. STM32 Feather SDIO is not working with stm32duino SD library. Seems related to pin configuration. Issue found here: https://github.com/adafruit/Adafruit-Feather-STM32F405-Express-PCB/issues/1. Will use SD breakout in the meantime.

## 2025-10-03
SD bench results on SPI:
write speed and latency
speed,max,min,avg
KB/Sec,usec,usec,usec
1118.57,1679,450,457
1119.32,11174,450,456

read speed and latency
speed,max,min,avg
KB/Sec,usec,usec,usec
1089.09,470,468,469
1089.80,470,468,469
