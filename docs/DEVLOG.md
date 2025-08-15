## 2025-08-15
Goal: Bring up LPS22HB and BNO085 on Feather F405 over I2C.
Issue: I2C scan showed no devices; SCL=0V.
Fix: Resolder SCL header on LPS22HB; scanner found Ox5C.
Next: SD SPI test
