#include <LionMotionSensor.h>

void LionMotionSensor::setPitchOffset(float offset) { offsetPitch = offset; }
void LionMotionSensor::setRollOffset(float offset) { offsetRoll = offset; }
void LionMotionSensor::setYawOffset(float offset) { offsetYaw = offset; }

void LionMotionSensor::begin() {
    Wire.begin();
    if(mpu.begin(MPU_SENSOR_ADDRESS)) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
        mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);
    }
}

void LionMotionSensor::loop(void(*onSensorDataCallback)(LionMotionSensorData)) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastExecutionTime;
    if (elapsedTime >= samplingTime) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        LionMotionSensorData sensorData;
        sensorData.pitch = calculatePitch(&a.acceleration);
        sensorData.roll = calculateRoll(&a.acceleration);
        sensorData.yaw = calculateYaw(sensorData.roll, sensorData.pitch);

        onSensorDataCallback(sensorData);

        lastExecutionTime = currentTime;
    }
}

float LionMotionSensor::calculatePitch(sensors_vec_t* acceleration) {
    float pitch = (atan2(-acceleration->x, sqrt(pow(acceleration->y, 2) + pow(acceleration->z, 2))) * 180.0 / PI) - offsetPitch;
    float result = constrain(pitch, -180, 180);
    return result;
}

float LionMotionSensor::calculateRoll(sensors_vec_t* acceleration) {
    float roll = (atan2(acceleration->y, acceleration->z) * 180.0 / PI) - offsetRoll;
    float result = constrain(roll, -180, 180);
    return result;
}

float LionMotionSensor::calculateYaw(float roll, float pitch) {
    float roll_rad = radians(roll); 
    float pitch_rad = radians(pitch); 

    float result = atan2(sin(roll_rad), cos(pitch_rad) * cos(roll_rad)) - offsetYaw;
    return result;
}