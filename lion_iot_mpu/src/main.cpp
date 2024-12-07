#include <Arduino.h>
#include <Lion.h>
#include <LionMotionSensor.h>
#include <Servo.h>
#include <PID_v1.h>

Lion lion = Lion();
LionMotionSensor motionSensor = LionMotionSensor();

#define CALIBRATION_TIME_SECS 5
int calibrationCounter = CALIBRATION_TIME_SECS;

#define MODE_INITIALIZING 1
#define MODE_CALIBRATING 2
#define MODE_RUNNING 3
#define MODE_SETUP 4
int robotStatus = MODE_INITIALIZING;

unsigned long lastCalibrationLoopTime;
double kp = 12.5;
double ki = 0.8;
double kd = 0.05;
double setPoint = 3.15;
double robotPitch = 0.00;
double robotPitchMaxPitchThreshold = 40.0; //Robot´s pitch value threshold to stop.

#define SETUP_P 1
#define SETUP_I 2
#define SETUP_D 3
#define SETUP_A 4
int setupMode = SETUP_P;
double setupFieldOffset = 0.1;
double* setupField = &kp;
String setupFieldName = "kP";

double pidOutput;  
PID pid = PID(&robotPitch, &pidOutput, &setPoint, kp, ki, kd, DIRECT);

Servo servoRight;
Servo servoLeft;

void printInitializingDisplay() {
  lion.display()->clear();
  lion.display()->drawLogo();
  lion.display()->writeAlignedText(55, 1, CENTER, "Press A to Start");
  lion.display()->draw();
}

void printCalibratingDisplay() {
  lion.display()->clear();
  lion.display()->writeAlignedText(13, 3, CENTER, (String)calibrationCounter);
  lion.display()->writeAlignedText(50, 1, CENTER, "(" + (String)setPoint + ") Calibrating...");
  lion.display()->draw();
}

void printRunningDisplay() {
  lion.display()->clear();
  lion.display()->writeAlignedText(22, 2, CENTER, (String)*setupField);
  lion.display()->writeAlignedText(40, 2, CENTER, setupFieldName);
  lion.display()->draw();
}

void printSetupDisplay() {
  int activeRow = 0;
  switch (setupMode) {
    case SETUP_P:
      activeRow = 5;
      break;
    case SETUP_I:
      activeRow = 20;
      break;
    case SETUP_D:
      activeRow = 35;
      break;
    case SETUP_A:
      activeRow = 50;
      break;
  }
  lion.display()->clear();
  lion.display()->writeText(10, 5, 1, "kP: " + (String)kp);
  lion.display()->writeText(10, 20, 1, "kI: " + (String)ki);
  lion.display()->writeText(10, 35, 1, "kD: " + (String)kd);
  lion.display()->writeText(10, 50, 1, "SP: " + (String)setPoint);
  lion.display()->writeText(0, activeRow, 1, "*");
  lion.display()->writeAlignedText(25, 2, RIGHT, (String)*setupField);
  lion.display()->writeAlignedText(42, 1, RIGHT, "-/+ " + (String)setupFieldOffset);
  lion.display()->draw();
}

void stop() {
  servoLeft.write(90);
  servoRight.write(90);
}

void moveForward(int speed) {
  int leftSpeed = 90 + speed;
  int rightSpeed = 90 - speed;
  servoLeft.write(leftSpeed);
  servoRight.write(rightSpeed);
}

void moveRear(int speed) {
  int leftSpeed = 90 - speed;
  int rightSpeed = 90 + speed;
  servoLeft.write(leftSpeed);
  servoRight.write(rightSpeed);
}

void attachMotors() {
  servoRight.attach(PORT_C);
  servoLeft.attach(PORT_D);
  stop();
}

void onMotionSenserData(LionMotionSensor::LionMotionSensorData sensorData) {
  switch (robotStatus) {
    case MODE_CALIBRATING:
      //setPoint = sensorData.pitch;
      break;
    case MODE_RUNNING:
      robotPitch = sensorData.pitch;
      break;
  }
}

void initialisePID() {
  pid = PID(&robotPitch, &pidOutput, &setPoint, kp, ki, kd, DIRECT);
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-90, 90);
}

void increaseSetupField() { 
  *setupField = *setupField + setupFieldOffset;
}

void decreaseSetupField() {
  *setupField = *setupField - setupFieldOffset;
}

void setupSetupField(int mode) {
  switch (mode) {
    case SETUP_P:
      setupFieldOffset = 0.1;
      setupField = &kp;
      setupFieldName = "kP";
      break;
    case SETUP_I:
      setupFieldOffset = 0.1;
      setupField = &ki;
      setupFieldName = "kI";
      break;
    case SETUP_D:
      setupFieldOffset = 0.01;
      setupField = &kd;
      setupFieldName = "kD";
      break;
    case SETUP_A:
      setupFieldOffset = 0.01;
      setupField = &setPoint;
      setupFieldName = "setPoint";
      break;
  }
}

void onTouchButtonEventForInitializing(TouchButtons::BUTTON button, TouchButtons::BUTTON_EVENT event) {
  if (button == TouchButtons::A && event == TouchButtons::PRESSED) {
      robotStatus = MODE_CALIBRATING;
  }
}

void onTouchButtonEventForCalibration(TouchButtons::BUTTON button, TouchButtons::BUTTON_EVENT event) { }

void onTouchButtonEventForRunning(TouchButtons::BUTTON button, TouchButtons::BUTTON_EVENT event) { 
  if (event == TouchButtons::BUTTON_EVENT::PRESSED) {
    switch (button) {
      case TouchButtons::BUTTON::A: 
        decreaseSetupField();
        initialisePID();
        break;
      case TouchButtons::BUTTON::B:
        increaseSetupField();
        initialisePID(); 
        break;
    }
    printRunningDisplay();
  } else if (event == TouchButtons::BUTTON_EVENT::LONG_PRESSED) {
    stop();
    robotStatus = MODE_SETUP;
  }
}

void onTouchButtonEventForSetup(TouchButtons::BUTTON button, TouchButtons::BUTTON_EVENT event) {
  switch (event) {
    case TouchButtons::BUTTON_EVENT::PRESSED:
      switch (button) {
        case TouchButtons::BUTTON::A:
          decreaseSetupField();
          initialisePID();
          break;
        case TouchButtons::BUTTON::B:
          increaseSetupField();
          initialisePID();
          break;
      }
      break;
    case TouchButtons::BUTTON_EVENT::LONG_PRESSED:
      switch (button) {
        case TouchButtons::BUTTON::A:
          if (setupMode < SETUP_A) {
            setupMode++;
          } else {
            setupMode = SETUP_P;
          }
          setupSetupField(setupMode);
          break;
        case TouchButtons::BUTTON::B:
          robotStatus = MODE_RUNNING;
          printRunningDisplay();
          break;
      }
      break;
  }
}

void onTouchButtonEvent(TouchButtons::BUTTON button, TouchButtons::BUTTON_EVENT event) {
  switch (robotStatus) {
    case MODE_INITIALIZING:
      onTouchButtonEventForInitializing(button, event);
      break;
    case MODE_CALIBRATING:
      onTouchButtonEventForCalibration(button, event);
      break;
    case MODE_RUNNING:
      onTouchButtonEventForRunning(button, event);
      break;
    case MODE_SETUP:
      onTouchButtonEventForSetup(button, event);
      break;
  }
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
      robotStatus = MODE_RUNNING;
      initialisePID();
      printRunningDisplay();
      calibrationCounter = CALIBRATION_TIME_SECS;
      lastCalibrationLoopTime = 0;
    }
    lastCalibrationLoopTime = currentTime;
  }
}

boolean isRobotPitchWithinMaxRange() {
  if (abs(robotPitch) <= robotPitchMaxPitchThreshold) {
    return true;
  } else {
    return false;
  }
}

int calculateMotorsSpeed(double pidValue) {
  return (int)pidValue;
} 

void runningLoop() {
  if (isRobotPitchWithinMaxRange()) {
    if (pid.Compute()) {
      int speed = calculateMotorsSpeed(pidOutput);
      if (speed < 0) {
        moveForward(abs(speed));
      } else if (speed > 0) {
        moveRear(abs(speed));
      } else {
        stop();
      }
    } else {
      stop();
    }
  } else {
    stop();
  }
}

void setupLoop() {
  printSetupDisplay();
}

void setup() {
  Serial.begin(9600);
  lion.begin();
  printInitializingDisplay();

  attachMotors();

  motionSensor.begin();
}

void loop() {
  lion.touchButtons()->loop(onTouchButtonEvent);

  motionSensor.loop(onMotionSenserData);

  switch (robotStatus) {
    case MODE_CALIBRATING:
      calibrationLoop();
      break;
    case MODE_RUNNING:
      runningLoop();
      break;
    case MODE_SETUP:
      setupLoop();
      break;
  }
}
