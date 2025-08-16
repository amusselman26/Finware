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
