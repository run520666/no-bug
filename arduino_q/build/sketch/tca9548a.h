#line 1 "D:\\GIT\\NO-BUG-QJ\\arduino_q\\tca9548a.h"
#ifndef TCA9548A_H
#define TCA9548A_H

#include <Arduino.h>
#include <Wire.h>

#define TCA9548A_ADDR 0x70

void tcaSelect(uint8_t channel);

#endif
