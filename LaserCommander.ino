#include "Laser.h"
// #include "LaserShowStuff.h"
#include "SerialComms.h"

#define PIN_RED 15
#define PIN_GRN 13
#define PIN_BLU 11

#define PIN_GALVO_X 4
#define PIN_GALVO_Y 2

Laser laser(PIN_GALVO_X, PIN_GALVO_Y, PIN_RED, PIN_GRN, PIN_BLU);
SerialComms* comms;

const int speed = 200; // Shared speed value

unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for serial port to connect
  delay(1);
  laser.init();
  laser.setOffset(0, 0);
  laser.setScale(1);
  comms = new SerialComms(laser);
}

void loop() {
  comms->processIncomingData();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.println("READY");
  }
}