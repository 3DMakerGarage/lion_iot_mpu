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

#ifndef LionMotionSensor_h
#define LionMotionSensor_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Deferred.h>

#define MPU_SENSOR_ADDRESS 0x68
#define MPU_SENSOR_SAMPLING_TIME 10

typedef void (*waitForCallback)();

class LionMotionSensor : private OnDeferredExecutionCallback {
     public:
        struct SensorData {
            float pitch = 0.0;
            float roll = 0.0;
            float yaw = 0.0;
        };
        
        typedef void(*OnSensorDataEvent)(SensorData);

        void setOnSensorDataEventCallback(OnSensorDataEvent callback);
        void setPitchOffset(float offset);
        void setRollOffset(float offset);
        void setYawOffset(float offset);
        void begin();
        void loop();

    private:
        float offsetPitch = 0.0f;
        float offsetRoll = 0.0f;
        float offsetYaw = 0.0f;
        unsigned long lastExecutionTime = 0;
        unsigned long samplingTime = MPU_SENSOR_SAMPLING_TIME; //1 Sec.
        Adafruit_MPU6050 mpu;
        Deferred deferredLoop = Deferred(10);
        OnSensorDataEvent onSensorDataEventCallback;        

        virtual void onDeferredExecution();
        float calculatePitch(sensors_vec_t* acceleration);
        float calculateRoll(sensors_vec_t* acceleration);
        float calculateYaw(float roll, float pitch);
};

#endif