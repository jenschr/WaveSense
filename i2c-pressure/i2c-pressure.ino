#include <Wire.h>
#include <i2cdetect.h>

byte data[2];
uint16_t pressure_bits = 0;
double pressure = 0.0;

void setup() {
  Wire.begin(3,4);
  Serial.begin(115200);
  delay(3000);
  Serial.println("Pressure sensor example");
}

void loop() {
  Wire.requestFrom(0x28, 2);
  data[0] = Wire.read();
  data[1] = Wire.read();
  
  pressure_bits = ((data[0] & 0b00111111) << 8) | data[1];
  
  Serial.println(pressure_bits, DEC);
  delay(50);
  return;
  pressure = ((pressure_bits - 1638) / (14745.0 - 1638.0)) * (2068.428 - 0.0);
  Serial.println(pressure, DEC);
}
