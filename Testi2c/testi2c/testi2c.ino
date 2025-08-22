#include <Wire.h>
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Escaneando I2C...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Encontrado: 0x"); Serial.println(addr, HEX);
    }
  }
  Serial.println("Fin de escaneo.");
}
void loop() {}
