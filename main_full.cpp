#include <Arduino.h>
#include <Lion.h>
#include <LionMotionSensor.h>
#include <Servo.h>
#include <PID_v1.h>

#define MODE_INITIALIZING 1
#define MODE_CALIBRATING 2
#define MODE_RUNNING 3
#define CALIBRATION_TIME_SECS 5
//#define ENABLE_MOTORS
//#define ENABLE_MPU

Lion lion = Lion();

int robotStatus = MODE_INITIALIZING;
int calibrationCounter = CALIBRATION_TIME_SECS;
unsigned long lastCalibrationLoopTime;
double kp = 15.0;
double ki = 0.0;
double kd = 0.0;

#ifdef ENABLE_MPU
  LionMotionSensor motionSensor = LionMotionSensor();
  float pitchOffset = 0.0;
  float currentPitch = 0.0;

  //PID Control
  double pidInput, pidOutput;
  double pidSetpoint = 0.0;

  PID pid(&pidInput, &pidOutput, &pidSetpoint, kp, ki, kd, DIRECT);
#endif

#ifdef ENABLE_MOTORS
  Servo servoRight;
  Servo servoLeft;
#endif

void printInitializingDisplay() {
  lion.display()->clear();
  lion.display()->writeAlignedText(15, 2, CENTER, "CALIBRATE");
  lion.display()->writeAlignedText(40, 1, CENTER, "Press A to Start");
  lion.display()->draw();
  delay(100);
}

void printCalibratingDisplay() {
  lion.display()->clear();
  lion.display()->writeAlignedText(13, 3, CENTER, (String)calibrationCounter);
  lion.display()->writeAlignedText(50, 1, CENTER, "Calibrating...");
  lion.display()->draw();
  delay(100);
}

void printRunningDisplay() {
  lion.display()->clear();
  lion.display()->writeAlignedText(20, 2, CENTER, "Kp: " + (String)kp);
  lion.display()->draw();
  delay(100);
}

#ifdef ENABLE_MOTORS
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
#endif 

#ifdef ENABLE_MPU
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
  void setCalibration() {
    motionSensor.setPitchOffset(pitchOffset);
  }
#endif

void increaseP() {
  if (kp < 100) {
    kp++;
  }
} 

void decreaseP() {
  if (kp > 0) {
    kp--;
  }
}  

void onTouchButtonEvent(TouchButtons::BUTTON button, TouchButtons::BUTTON_EVENT event) {
  //if (TouchButtons::PRESSED && robotStatus == MODE_RUNNING) {
  if (event == TouchButtons::BUTTON_EVENT::PRESSED) {
    switch (button) {
      case TouchButtons::BUTTON::A: 
        decreaseP();
        break;
      case TouchButtons::BUTTON::B:
        increaseP(); 
        break;
    }
    printRunningDisplay();
  }


//  if (button == TouchButtons::A && event == TouchButtons::PRESSED) {
//    if (robotStatus == MODE_INITIALIZING) {
//      robotStatus = MODE_CALIBRATING;
//    }
//  }
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
#ifdef ENABLE_MPU
      setCalibration();
#endif
      calibrationCounter = CALIBRATION_TIME_SECS;
      lastCalibrationLoopTime = 0;
      robotStatus = MODE_RUNNING;
    }
    lastCalibrationLoopTime = currentTime;
  }
}

void runningLoop() {
#ifdef ENABLE_MPU
  pidInput = currentPitch;
  pid.Compute();
  //Serial.println((String)pidOutput);
#endif

#ifdef ENABLE_MOTORS
  unsigned int speed = abs(int(trunc(pidOutput)));
  if (pidOutput < 0.0) {
    moveRear(speed);
  } else if (pidOutput > 0.0) {
    moveForward(speed);
  } else if (pidOutput == 0.0) {
    stop();
  }
#endif
}

#ifdef ENABLE_MPU
  void setupPID() {
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-90, 90);
  }
#endif

#ifdef ENABLE_MOTORS
  void attachMotors() {
    servoRight.attach(PORT_D);
    servoLeft.attach(PORT_C);
    stop();
  }
#endif

void setup() {
  Serial.begin(9600);
  lion.begin();
  //printInitializingDisplay();

#ifdef ENABLE_MOTORS
  attachMotors();
#endif

#ifdef ENABLE_MPU
  setupPID();
  motionSensor.begin();
#endif
}

void loop() {
  lion.touchButtons()->loop(onTouchButtonEvent);

#ifdef ENABLE_MPU
  motionSensor.loop(onMotionSenserData);
#endif
/*
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
*/
}
