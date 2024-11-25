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
double kp = 30.0;
double ki = 2.0;
double kd = 0.0;
double setPoint = 0.0;
double robotPitch = 0.0;

#define SETUP_P 1
#define SETUP_I 2
#define SETUP_D 3
#define SETUP_A 4
int setupMode = SETUP_P;
double setupFieldOffset = 1.0;
double* setupField = &kp;
String setupFieldName = "kP";

double pidOutput;  
PID pid = PID(&robotPitch, &pidOutput, &setPoint, kp, ki, kd, DIRECT);

Servo servoRight;
Servo servoLeft;

void printInitializingDisplay() {
  lion.display()->clear();
  lion.display()->writeAlignedText(15, 2, CENTER, "CALIBRATE");
  lion.display()->writeAlignedText(40, 1, CENTER, "Press A to Start");
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
  lion.display()->writeText(10, 5, 1, "P: " + (String)kp);
  lion.display()->writeText(10, 20, 1, "I: " + (String)ki);
  lion.display()->writeText(10, 35, 1, "D: " + (String)kd);
  lion.display()->writeText(10, 50, 1, "A: " + (String)setPoint);
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
  int leftSpeed = 90 - speed;
  int rightSpeed = 90 + speed;
  servoLeft.write(leftSpeed);
  servoRight.write(rightSpeed);
}

void moveRear(int speed) {
  int leftSpeed = 90 + speed;
  int rightSpeed = 90 - speed;
  servoLeft.write(leftSpeed);
  servoRight.write(rightSpeed);
}

void onMotionSenserData(LionMotionSensor::LionMotionSensorData sensorData) {
  switch (robotStatus) {
    case MODE_CALIBRATING:
      setPoint = sensorData.pitch;
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
  if (*setupField > 0.0) {
    *setupField = *setupField - setupFieldOffset;
  }
}

void setupSetupField(int mode) {
  switch (mode) {
    case SETUP_P:
      setupFieldOffset = 1.0;
      setupField = &kp;
      setupFieldName = "kP";
      break;
    case SETUP_I:
      setupFieldOffset = 0.1;
      setupField = &ki;
      setupFieldName = "kI";
      break;
    case SETUP_D:
      setupFieldOffset = 0.1;
      setupField = &kd;
      setupFieldName = "kD";
      break;
    case SETUP_A:
      setupFieldOffset = 0.1;
      setupField = &setPoint;
      setupFieldName = "SetPoint";
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

void runningLoop() {
  if (pid.Compute()) {
    unsigned int speed = abs(int(trunc(pidOutput)));
    if (robotPitch < 0.0) {
      moveForward(speed);
    } else if (robotPitch > 0.0) {
      moveRear(speed);
    } else if (robotPitch == 0.0) {
      stop();
    }
  }
}

void setupLoop() {
  printSetupDisplay();
}

void attachMotors() {
  servoRight.attach(PORT_D);
  servoLeft.attach(PORT_E);
  stop();
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
    case MODE_INITIALIZING:
      break;
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
