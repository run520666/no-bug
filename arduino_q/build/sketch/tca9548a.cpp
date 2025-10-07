#line 1 "D:\\GIT\\NO-BUG-QJ\\arduino_q\\tca9548a.cpp"
#include "tca9548a.h"

void tcaSelect(uint8_t channel) {
  if (channel > 7) {
    Serial.print("Invalid TCA channel: ");
    Serial.println(channel);
    return;
  }
  
  Wire.beginTransmission(TCA9548A_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("TCA9548A not found!");
    return;
  }
  
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) {
    Serial.print("Failed to select TCA channel ");
    Serial.println(channel);
  }
  delay(20);
}
