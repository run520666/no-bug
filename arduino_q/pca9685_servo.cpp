#include "pca9685_servo.h"
#include "tca9548a.h"

#define TCA_PCA9685_CH 2

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

bool pca9685Init() {
  tcaSelect(TCA_PCA9685_CH);
  delay(50);
  
  if (!pca.begin()) {
    return false;
  }
  pca.setPWMFreq(50);
  return true;
}

uint16_t relativeAngleToPulseSG90(int relativeAngle) {
  int angle = map(SG90_HOME_PULSE, SG90_MIN_PULSE, SG90_MAX_PULSE, 0, 180);
  angle += relativeAngle;
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SG90_MIN_PULSE, SG90_MAX_PULSE);
}

void controlSG90Relative(int channel, int relativeAngle) {
  tcaSelect(TCA_PCA9685_CH);
  uint16_t pulse = relativeAngleToPulseSG90(relativeAngle);
  pca.setPWM(channel, 0, pulse);
  delay(100);
}

void resetSG90(int channel) {
  controlSG90Relative(channel, 0);
}
