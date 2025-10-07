#ifndef GM65_QRCODE_H
#define GM65_QRCODE_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#define GM65_1_RX 10
#define GM65_1_TX 11
#define GM65_2_RX 12
#define GM65_2_TX 13
#define GM65_BAUD 9600
#define GM65_TIMEOUT 100

extern SoftwareSerial gm65_1;
extern SoftwareSerial gm65_2;

void gm65Init();
String readGM65Data(SoftwareSerial &gm65);
void processGM65Data(String data, int gm65Num);

#endif
