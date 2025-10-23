#include "gm65_qrcode.h"
#include "color_sensor_servo.h"
#include "pca9685_servo.h"

SoftwareSerial gm65_1(GM65_1_RX, GM65_1_TX);
SoftwareSerial gm65_2(GM65_2_RX, GM65_2_TX);

void gm65Init() {
  gm65_1.begin(GM65_BAUD);
  gm65_2.begin(GM65_BAUD);
}

String readGM65Data(SoftwareSerial &gm65) {
  String data = "";
  unsigned long startTime = millis();
  
  while (!gm65.available() && millis() - startTime < GM65_TIMEOUT) {
    delay(1);
  }
  
  while (gm65.available()) {
    data += (char)gm65.read();
    delay(2);
  }
  
  return data;
}

void processGM65Data(String data, int gm65Num) {
  if (!isMappingDone) {
    Serial.println("Error: Mapping Not Completed!");
    return;
  }

  if (data.length() == 0) return;
  Serial.print("GM65 ");
  Serial.print(gm65Num);
  Serial.print(" Read: ");
  Serial.println(data);

  char targetQrChar = ' ';
  for (int i = 0; i < data.length(); i++) {
    char c = tolower(data[i]);
    for (int q = 0; q < COLOR_QR_COUNT; q++) {
      if (c == colorQrMap[q].qrCodeChar) {
        targetQrChar = c;
        break;
      }
    }
    if (targetQrChar != ' ') break;
  }

  const char* targetColor = "Unknown";
  for (int q = 0; q < COLOR_QR_COUNT; q++) {
    if (colorQrMap[q].qrCodeChar == targetQrChar) {
      targetColor = colorQrMap[q].colorName;
      break;
    }
  }

  int targetChannels[2];
  int channelCount = getChannelsByColor(targetColor, targetChannels);
  if (channelCount == 0) {
    Serial.println("Error: No Channels Found!");
    return;
  }

  Serial.print("Release All ");
  Serial.print(targetColor);
  Serial.print(" Balls (Channels: ");
  for (int i = 0; i < channelCount; i++) {
    Serial.print(targetChannels[i]);
    if (i < channelCount - 1) Serial.print(", ");
    controlSG90Relative(targetChannels[i], SG90_RELATIVE_ANGLE);
    delay(500);
    resetSG90(targetChannels[i]);
  }
  Serial.println(")");
}
