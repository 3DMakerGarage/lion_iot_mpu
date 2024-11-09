#ifndef MPU_h
#define MPU_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define MPU_SENSOR_ADDRESS 0x68
#define MPU_SENSOR_SAMPLING_TIME 100

typedef void (*waitForCallback)();

class LionMotionSensor {
private:
    float offsetPitch = 0.0f;
    float offsetRoll = 0.0f;
    float offsetYaw = 0.0f;
    unsigned long lastExecutionTime = 0;
    unsigned long samplingTime = MPU_SENSOR_SAMPLING_TIME; //1 Sec.
    Adafruit_MPU6050 mpu;

    float calculatePitch(sensors_vec_t* acceleration);
    float calculateRoll(sensors_vec_t* acceleration);
    float calculateYaw(float roll, float pitch);

public:
    struct LionMotionSensorData {
        float pitch = 0.0;
        float roll = 0.0;
        float yaw = 0.0;
    };
    
    void setPitchOffset(float offset);
    void setRollOffset(float offset);
    void setYawOffset(float offset);
    void begin();
    void loop(void(*onSensorDataCallback)(LionMotionSensorData));
};

#endif