#include <Arduino.h>
#include <Lion.h>
#include <LionMotionSensor.h>
#include <Servo.h>
#include <PID_v1.h>

#define MODE_INITIALIZING 1
#define MODE_CALIBRATING 2
#define MODE_RUNNING 3
#define CALIBRATION_TIME_SECS 5
#define ENABLE_MOTORS

Lion lion = Lion();
LionMotionSensor motionSensor = LionMotionSensor();
int robotStatus = MODE_INITIALIZING;
int calibrationCounter = CALIBRATION_TIME_SECS;
unsigned long lastCalibrationLoopTime;
float pitchOffset = 0.0;
float currentPitch = 0.0;
Servo servoRight;
Servo servoLeft;

//PID Control
double pidInput, pidOutput;
double pidSetpoint = 0.0;
double kp = 15.0;
double ki = 0.0;
double kd = 0.0;
PID pid(&pidInput, &pidOutput, &pidSetpoint, kp, ki, kd, DIRECT);

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
  #ifdef ENABLE_MOTORS
    servoLeft.write(90);
    servoRight.write(90);
  #endif
}

void moveForward(int speed) {
  #ifdef ENABLE_MOTORS
    int leftSpeed = 90 - speed;
    int rightSpeed = 90 + speed;
    servoLeft.write(leftSpeed);
    servoRight.write(rightSpeed);
  #endif
}

void moveRear(int speed) {
  #ifdef ENABLE_MOTORS
    int leftSpeed = 90 + speed;
    int rightSpeed = 90 - speed;
    servoLeft.write(leftSpeed);
    servoRight.write(rightSpeed);
  #endif
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
  } else if (robotStatus == MODE_RUNNING) {
    currentPitch = roundf(sensorData.pitch);
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
  pidInput = currentPitch;
  pid.Compute();
  Serial.println((String)pidOutput);
  
  unsigned int speed = abs(int(trunc(pidOutput)));
  if (pidOutput < 0.0) {
    moveRear(speed);
  } else if (pidOutput > 0.0) {
    moveForward(speed);
  } else if (pidOutput == 0.0) {
    stop();
  }
}

void setupPID() {
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-90, 90);
}

void setup() {
  Serial.begin(115200);
  lion.begin();
  printInitializingDisplay();

  servoRight.attach(PORT_D);
  servoLeft.attach(PORT_C);
  stop();

  setupPID();
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
