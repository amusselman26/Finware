#include <Wire.h>
void setup() {
  Serial.begin(115200); delay(2000);
  Wire.begin(); Wire.setClock(100000); delay(20);
  Serial.println("I2C scan...");
}
void loop() {
  uint8_t n=0;
  for (uint8_t a=1; a<127; a++){
    Wire.beginTransmission(a);
    if (Wire.endTransmission()==0){
      Serial.print("Found 0x"); if(a<16) Serial.print('0'); Serial.println(a,HEX); n++;
    }
  }
  if(!n) Serial.println("No I2C devices found");
  Serial.println("----");
  delay(1500);
}
