#include <Arduino.h>
#include <MPU.h>
#include <Lion.h>

Lion lion;
MPU mpuSensor = MPU();

void setup() {
  Serial.begin(115200);
  lion.begin();

  mpuSensor.setPitchOffset(-1.05);
  mpuSensor.setRollOffset(1.9);
  mpuSensor.setYawOffset(0.3);
  mpuSensor.begin();
}

void loop() {
  mpuSensor.loop();
}