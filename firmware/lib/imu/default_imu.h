// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

//include IMU base interface
#include "imu_interface.h"

//include sensor API headers
#include "I2Cdev.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MPU9150.h"
#include "MPU9250.h"

class GY85IMU: public IMUInterface 
{
    private:
        //constants specific to the sensor
        const float accel_scale_ = 1 / 256.0;
        const float gyro_scale_ = 1 / 14.375;

        // driver objects to be used
        ADXL345 accelerometer_;
        ITG3200 gyroscope_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;
        geometry_msgs__msg__Vector3 mag_;

    public:
        GY85IMU()
        {
            // accel_cov_ = 0.001; //you can overwrite the convariance values here
            // gyro_cov_ = 0.001; //you can overwrite the convariance values here
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from accelerometer and return as a Vector3 object
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from gyroscope and return as a Vector3 object
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            mag_ = {0, 0, 0};
            return mag_;
        }


        void setAccelCalib(float* scale, float* bias) override
        {
        }

        void calibrateMag(const linorobot2_interfaces__srv__CalibrateMag_Request* req,
                                linorobot2_interfaces__srv__CalibrateMag_Response* res,
                                void (*moveCallback)()) override
        {
        }

        std::string readErrorStr() override
        {
            return "";
        }

};


class MPU6050IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU6050 accelerometer_;
        MPU6050 gyroscope_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;
        geometry_msgs__msg__Vector3 mag_;

    public:
        MPU6050IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            mag_ = {0, 0, 0};
            return mag_;
        }

        void calibrateMag(const linorobot2_interfaces__srv__CalibrateMag_Request* req,
                                linorobot2_interfaces__srv__CalibrateMag_Response* res,
                                void (*moveCallback)()) override
        {
        }

        void setAccelCalib(float* scale, float* bias) override
        {
        }

        std::string readErrorStr() override
        {
            return "";
        }
};

class MPU9150IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU9150 accelerometer_;
        MPU9150 gyroscope_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;
        geometry_msgs__msg__Vector3 mag_;

    public:
        MPU9150IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            mag_ = {0, 0, 0};
            return mag_;
        }        

        void calibrateMag(const linorobot2_interfaces__srv__CalibrateMag_Request* req,
                                linorobot2_interfaces__srv__CalibrateMag_Response* res,
                                void (*moveCallback)()) override
        {
        }
        
        void setAccelCalib(float* scale, float* bias) override
        {
        }
        
        std::string readErrorStr() override
        {
            return "";
        }
};

class MPU9250IMU: public IMUInterface
{    
    private:     

        #define I2Cclock 400000
        #define I2Cport Wire
        
        #define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
        //#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

        MPU9250 mpu9250_;
        float accelBias[3] = {0, 0, 0};
        float accelScale[3] = {1, 1, 1};

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;
        geometry_msgs__msg__Vector3 mag_;

        std::string errStr;
        
    public:
        MPU9250IMU() : mpu9250_(MPU9250_ADDRESS, I2Cport, I2Cclock)
        {
            accel_cov_ = 0.0016;
            gyro_cov_ = 0.0001;
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            // Read the WHO_AM_I register, this is a good test of communication
            byte device_id = mpu9250_.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
            ret = device_id == 0x38 || device_id == 0x71 || device_id == 0x39 || device_id == 0x73;
            if(!ret)
            {
                char errStrBuf[100];
                snprintf(errStrBuf, 99, "Device ID not recognized as MPU9250: 0x%x", device_id);
                errStr = errStrBuf;
                return false;
            }
                    
            // Start by performing self test and reporting values
            mpu9250_.MPU9250SelfTest(mpu9250_.selfTest);
            // Check if mpu9250_.selfTest values are <= 14 and >= -14
            if (mpu9250_.selfTest[0] <= 14 && mpu9250_.selfTest[0] >= -14 &&
                mpu9250_.selfTest[1] <= 14 && mpu9250_.selfTest[1] >= -14 &&
                mpu9250_.selfTest[2] <= 14 && mpu9250_.selfTest[2] >= -14 &&
                mpu9250_.selfTest[3] <= 14 && mpu9250_.selfTest[3] >= -14 &&
                mpu9250_.selfTest[4] <= 14 && mpu9250_.selfTest[4] >= -14 &&
                mpu9250_.selfTest[5] <= 14 && mpu9250_.selfTest[5] >= -14)
            {
                // Self-test passed
            }
            else
            {
                errStr = "MPU9250 Self test failed";
                return false;
            }

            // Calibrate gyro, load biases in bias registers. Don't calibrate accel because this call requires
            // that it be perfectly level to ground and it will not in most robots. Scale and Bias values are 
            // precalculated with 6 position calibration procedure with the IMU mounted in a calibration cube.
            mpu9250_.calibrateMPU9250(mpu9250_.gyroBias, nullptr);

            // Initialize device for active mode read of acclerometer, gyroscope, and
            // temperature
            mpu9250_.initMPU9250();

            // Read the WHO_AM_I register of the magnetometer, this is a good test of
            // communication
            byte device2_id = mpu9250_.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
            if (device2_id != 0x48) 
            {
                char errStrBuf[100];
                snprintf(errStrBuf, 99, "Device ID not recognized as AK8963: 0x%x", device2_id);                
                errStr = errStrBuf;
                return false;
            }

            // Get magnetometer calibration from AK8963 ROM
            // Initialize device for active mode read of magnetometer
            mpu9250_.initAK8963(mpu9250_.factoryMagCalibration);

            // Get sensor resolutions, only need to do this once
            mpu9250_.getAres();
            mpu9250_.getGres();
            mpu9250_.getMres();
            
            return true;
        }

        // Per ROS body frame standard x is forward, y is left, z is up
        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            
            mpu9250_.readAccelData(mpu9250_.accelCount);  // Read the x/y/z adc values

            // Now we'll calculate the accleration value into actual m/s^2
            // This depends on scale being set
            accel_.x = (mpu9250_.accelCount[0] * (double) mpu9250_.aRes * g_to_accel_ - accelBias[0]) * accelScale[0];
            accel_.y = (mpu9250_.accelCount[1] * (double) mpu9250_.aRes * g_to_accel_ - accelBias[1]) * accelScale[1];
            accel_.z = (mpu9250_.accelCount[2] * (double) mpu9250_.aRes * g_to_accel_ - accelBias[2]) * accelScale[2];

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            mpu9250_.readGyroData(mpu9250_.gyroCount);  // Read the x/y/z adc values

            // Calculate the gyro value into actual radians per second
            // This depends on scale being set
            gyro_.x = mpu9250_.gyroCount[0] * (double) mpu9250_.gRes * DEG_TO_RAD;
            gyro_.y = mpu9250_.gyroCount[1] * (double) mpu9250_.gRes * DEG_TO_RAD;
            gyro_.z = mpu9250_.gyroCount[2] * (double) mpu9250_.gRes * DEG_TO_RAD;

            return gyro_;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            mpu9250_.readMagData(mpu9250_.magCount);  // Read the x/y/z adc values

            // Calculate the magnetometer values in teslas
            // Include factory calibration per data sheet and user environmental
            // corrections
            // Get actual magnetometer value, this depends on scale being set
            
            // swap x and y and invert z to switch to ENU inertial frame required by ROS
            // https://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions
            double temp_x = (mpu9250_.magCount[0] * mpu9250_.mRes * mpu9250_.factoryMagCalibration[0] - mpu9250_.magBias[0]) *
                mpu9250_.magScale[0] * (double) mgauss_to_tesla_;
            double temp_y = (mpu9250_.magCount[1] * mpu9250_.mRes * mpu9250_.factoryMagCalibration[1] - mpu9250_.magBias[1]) *
                mpu9250_.magScale[1] * (double) mgauss_to_tesla_;
            mag_.x = temp_y;
            mag_.y = temp_x;
            mag_.z = -(mpu9250_.magCount[2] * mpu9250_.mRes * mpu9250_.factoryMagCalibration[2] - mpu9250_.magBias[2]) * 
                mpu9250_.magScale[2] * (double) mgauss_to_tesla_;
            
            return mag_;
        }
        
        void calibrateMag(float mag_bias[3], float mag_scale[3])
        {
            mpu9250_.magBias[0] = mag_bias[0];
            mpu9250_.magBias[1] = mag_bias[1];
            mpu9250_.magBias[2] = mag_bias[2];

            mpu9250_.magScale[0] = mag_scale[0];
            mpu9250_.magScale[1] = mag_scale[1];
            mpu9250_.magScale[2] = mag_scale[2];
        }

        void calibrateMag(const linorobot2_interfaces__srv__CalibrateMag_Request* req,
                                linorobot2_interfaces__srv__CalibrateMag_Response* res,
                                void (*moveCallback)()) override
        {
            // The next call delays for 4 seconds, and then records about 15 seconds of
            // data to calculate bias and scale.
            if (req->mag_scale.x == 0 && req->mag_scale.y == 0 && req->mag_scale.z == 0 
                || req->mag_bias.x == 0 && req->mag_bias.y == 0 && req->mag_bias.z == 0)
                mpu9250_.magCalMPU9250(mpu9250_.magBias, mpu9250_.magScale, moveCallback);
            else
            {
                mpu9250_.magBias[0] = req->mag_bias.x;
                mpu9250_.magBias[1] = req->mag_bias.y;
                mpu9250_.magBias[2] = req->mag_bias.z;

                mpu9250_.magScale[0] = req->mag_scale.x;
                mpu9250_.magScale[1] = req->mag_scale.y;
                mpu9250_.magScale[2] = req->mag_scale.z;
            }

            res->mag_bias.x = mpu9250_.magBias[0];
            res->mag_bias.y = mpu9250_.magBias[1];
            res->mag_bias.z = mpu9250_.magBias[2];

            res->mag_scale.x = mpu9250_.magScale[0];
            res->mag_scale.y = mpu9250_.magScale[1];
            res->mag_scale.z = mpu9250_.magScale[2];
        }

        void setAccelCalib(float* scale, float* bias) override
        {
            accelBias[0] = bias[0];
            accelBias[1] = bias[1];
            accelBias[2] = bias[2];
            accelScale[0] = scale[0];
            accelScale[1] = scale[1];
            accelScale[2] = scale[2];
        }

        std::string readErrorStr() override
        {
            return errStr;
        }        
};

class FakeIMU: public IMUInterface 
{
    private:
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;
        geometry_msgs__msg__Vector3 mag_;

    public:
        FakeIMU()
        {
        }

        bool startSensor() override
        {
            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            return gyro_;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            return mag_;
        }

        std::string readErrorStr() override
        {
            return "";
        }

        void calibrateMag(const linorobot2_interfaces__srv__CalibrateMag_Request* req,
                                linorobot2_interfaces__srv__CalibrateMag_Response* res,
                                void (*moveCallback)()) override
        {
        }

        void setAccelCalib(float* scale, float* bias) override
        {
        }
};

#endif
//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//HMC8553L https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf


//MPU9150 https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//MPU9250 https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf

//http://www.sureshjoshi.com/embedded/invensense-imus-what-to-know/
//https://stackoverflow.com/questions/19161872/meaning-of-lsb-unit-and-unit-lsb