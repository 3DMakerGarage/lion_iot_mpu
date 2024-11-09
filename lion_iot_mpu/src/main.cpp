#include <Arduino.h>
#include <Lion.h>
#include <LionMotionSensor.h>
#include <Servo.h>

#define MODE_INITIALIZING 1
#define MODE_CALIBRATING 2
#define MODE_RUNNING 3
#define CALIBRATION_TIME_SECS 10

Lion lion = Lion();
LionMotionSensor motionSensor = LionMotionSensor();
int robotStatus = MODE_INITIALIZING;
int calibrationCounter = CALIBRATION_TIME_SECS;
unsigned long lastCalibrationLoopTime;
float pitchOffset = 0.0;
float currentPitch = 0.0;
Servo servoRight;
Servo servoLeft;

void printInitializingDisplay() {
  lion.display()->clear();
  lion.display()->writeAlignedText(15, 2, CENTER, "CALIBRATE");
  lion.display()->writeAlignedText(40, 1, CENTER, "Press A to Start");
}

void printCalibratingDisplay() {
  lion.display()->clear();
  lion.display()->writeAlignedText(13, 3, CENTER, (String)calibrationCounter);
  lion.display()->writeAlignedText(50, 1, CENTER, "Calibrating...");
}

void printRunningDisplay() {
  lion.display()->clear();
  lion.display()->writeAlignedText(20, 3, CENTER, "RUN!!!");
}

void stop() {
  servoLeft.write(90);
  servoRight.write(90);
}

void moveForward() {
  servoLeft.write(180);
  servoRight.write(0);
}

void moveRear() {
  servoLeft.write(0);
  servoRight.write(180);
}

void onMotionSenserData(LionMotionSensor::LionMotionSensorData sensorData) {
  if (robotStatus == MODE_CALIBRATING && calibrationCounter <= CALIBRATION_TIME_SECS - 2) {
    if (pitchOffset != 0.0) {
      if (abs(sensorData.pitch) > abs(pitchOffset)) {
        pitchOffset = sensorData.pitch;  
      }
    } else {
      pitchOffset = sensorData.pitch;
    }
    Serial.println(sensorData.pitch);
  } else if (robotStatus == MODE_RUNNING) {
    currentPitch = roundf(sensorData.pitch);
    //Serial.println(currentPitch);
  }
}

void onTouchButtonEvent(TouchButtons::BUTTON button, TouchButtons::BUTTON_EVENT event) {
  if (button == TouchButtons::A && event == TouchButtons::PRESSED) {
    if (robotStatus == MODE_INITIALIZING) {
      robotStatus = MODE_CALIBRATING;
    }
  }
}

void setCalibration() {
  Serial.println("Robot Calibration:");
  Serial.println("\tPitch. Offset: " + (String)pitchOffset);
  motionSensor.setPitchOffset(pitchOffset);
}

void calibrationLoop() {
  unsigned long currentTime = millis();
  if (lastCalibrationLoopTime == 0) { lastCalibrationLoopTime = currentTime; }
  unsigned long elapsedTime = currentTime - lastCalibrationLoopTime;

  if (elapsedTime >= 1000) {
    if (calibrationCounter > 0) {
      printCalibratingDisplay();
      calibrationCounter--;
    } else {
      printRunningDisplay();
      setCalibration();
      calibrationCounter = CALIBRATION_TIME_SECS;
      lastCalibrationLoopTime = 0;
      robotStatus = MODE_RUNNING;
    }
    lastCalibrationLoopTime = currentTime;
  }
}

void runningLoop() {
  Serial.println(currentPitch);
  if (currentPitch < 0.0) {
    Serial.println("Moving Rear...");
    moveRear();
  } else if (currentPitch > 0.0) {
    Serial.println("Moving Forward...");
    moveForward();
  } else if (currentPitch == 0.0) {
    Serial.println("Stoping...");
    stop();
  }
}

void setup() {
  pinMode(PORT_D, OUTPUT);
  pinMode(PORT_E, OUTPUT);

  Serial.begin(115200);
  lion.begin();
  printInitializingDisplay();

  servoRight.attach(PORT_D);
  servoLeft.attach(PORT_E);
  stop();

  motionSensor.begin();
}

void loop() {
  lion.touchButtons()->loop(onTouchButtonEvent);
  motionSensor.loop(onMotionSenserData);

  switch (robotStatus) {
    case MODE_INITIALIZING:
      break;
    case MODE_CALIBRATING:
      calibrationLoop();
      break;
    case MODE_RUNNING:
      runningLoop();
      break;
  }
}
