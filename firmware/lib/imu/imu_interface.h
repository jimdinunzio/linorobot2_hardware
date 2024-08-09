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

#ifndef IMU_INTERFACE
#define IMU_INTERFACE

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <linorobot2_interfaces/srv/calibrate_mag.h>
#include <string>

class IMUInterface
{
    protected:
        sensor_msgs__msg__Imu imu_msg_;
        const float g_to_accel_ = 9.80665;
        const float mgauss_to_tesla_ = 0.0000001;
        
        float accel_cov_ = 0.00001;
        float gyro_cov_ = 0.00001;
        const int sample_size_ = 40;

        geometry_msgs__msg__Vector3 gyro_cal_ = {0};

        void calibrateGyro()
        {
            geometry_msgs__msg__Vector3 gyro;

            for(int i=0; i<sample_size_; i++)
            {
                gyro = readGyroscope();
                gyro_cal_.x += gyro.x;
                gyro_cal_.y += gyro.y;
                gyro_cal_.z += gyro.z;

                delay(50);
            }

            gyro_cal_.x = gyro_cal_.x / (float)sample_size_;
            gyro_cal_.y = gyro_cal_.y / (float)sample_size_;
            gyro_cal_.z = gyro_cal_.z / (float)sample_size_;
        }
   
    public:
        IMUInterface()
        {
            imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");
        }

        virtual geometry_msgs__msg__Vector3 readAccelerometer() = 0;
        virtual geometry_msgs__msg__Vector3 readGyroscope() = 0;
        virtual geometry_msgs__msg__Vector3 readMagnetometer() = 0;
        virtual void calibrateMag(const linorobot2_interfaces__srv__CalibrateMag_Request* req,
                                linorobot2_interfaces__srv__CalibrateMag_Response* res, 
                                void (*moveCallback)()) = 0;

        virtual void setAccelCalib(float* scale, float* bias) = 0;
        virtual std::string readErrorStr() = 0;

        virtual bool startSensor() = 0;

        bool init()
        {
            bool sensor_ok = startSensor();
            if(sensor_ok)
                calibrateGyro();

            return sensor_ok;
        }

        std::string getErrorStr()
        {
            return readErrorStr();
        }

        sensor_msgs__msg__MagneticField getMagneticField()
        {
            sensor_msgs__msg__MagneticField mag_msg;
            mag_msg.magnetic_field = readMagnetometer();
            mag_msg.magnetic_field_covariance[0] = 0.00001;
            mag_msg.magnetic_field_covariance[4] = 0.00001;
            mag_msg.magnetic_field_covariance[8] = 0.00001;

            return mag_msg;
        }

        sensor_msgs__msg__Imu getData()
        {
            imu_msg_.angular_velocity = readGyroscope();
            imu_msg_.angular_velocity.x -= gyro_cal_.x; 
            imu_msg_.angular_velocity.y -= gyro_cal_.y; 
            imu_msg_.angular_velocity.z -= gyro_cal_.z; 

            if(imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01 )
                imu_msg_.angular_velocity.x = 0; 
         
            if(imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01 )
                imu_msg_.angular_velocity.y = 0;

            if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01 )
                imu_msg_.angular_velocity.z = 0;
       
            imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[8] = gyro_cov_;
            
            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

            return imu_msg_;
        }
};

#endif
