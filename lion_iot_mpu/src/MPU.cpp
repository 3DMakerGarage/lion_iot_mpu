#include <MPU.h>

void MPU::setPitchOffset(float offset) { offsetPitch = offset; }
void MPU::setRollOffset(float offset) { offsetRoll = offset; }
void MPU::setYawOffset(float offset) { offsetYaw = offset; }

void MPU::begin() {
    Wire.begin();
    if(mpu.begin(MPU_SENSOR_ADDRESS)) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
        mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);
    }
}

void MPU::loop() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastExecutionTime;
    if (elapsedTime >= samplingTime) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float pitch = calculatePitch(&a.acceleration);
        float roll = calculateRoll(&a.acceleration);
        float yaw = calculateYaw(roll, pitch);

        Serial.print(pitch);
        Serial.print(",");
        Serial.print(roll);
        Serial.print(",");
        Serial.println(yaw);

        lastExecutionTime = currentTime;
    }
}

float MPU::calculatePitch(sensors_vec_t* acceleration) {
    float result = (atan2(-acceleration->x, sqrt(pow(acceleration->y, 2) + pow(acceleration->z, 2))) * 180.0 / PI) - offsetPitch;
    if (result > 180) {
        result -= 360;
    }
    return result;
}

float MPU::calculateRoll(sensors_vec_t* acceleration) {
    float result = (atan2(acceleration->y, acceleration->z) * 180.0 / PI) - offsetRoll;
    if (result > 180) {
        result -= 360;
    }
    return result;
}

float MPU::calculateYaw(float roll, float pitch) {
    float roll_rad = radians(roll); 
    float pitch_rad = radians(pitch); 

    float result = atan2(sin(roll_rad), cos(pitch_rad) * cos(roll_rad)) - offsetYaw;
    return result;
}