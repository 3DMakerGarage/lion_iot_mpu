/*
 * Copyright 3D Maker Garage 2024
 *
 * This file is part of some open source application.
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 * Contact: 3dmakergarage@gmail.com
 */

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

void LionMotionSensor::setOnSensorDataEventCallback(OnSensorDataEvent callback) {
    onSensorDataEventCallback = callback;
}

void LionMotionSensor::onDeferredExecution() {
    if (onSensorDataEventCallback) {
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - lastExecutionTime;
        if (elapsedTime >= samplingTime) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            SensorData sensorData;
            sensorData.pitch = calculatePitch(&a.acceleration);
            sensorData.roll = calculateRoll(&a.acceleration);
            sensorData.yaw = calculateYaw(sensorData.roll, sensorData.pitch);

            onSensorDataEventCallback(sensorData);

            lastExecutionTime = currentTime;
        }
    }
}

void LionMotionSensor::loop() {
    deferredLoop.execute(this);
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