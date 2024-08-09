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
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/service.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <linorobot2_interfaces/srv/calibrate_mag.h>

#include "config.h"
#include "logger.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
//#define ENCODER_USE_INTERRUPTS
//#define ENCODER_OPTIMIZE_INTERRUPTS
//#include "encoder.h"
#include "util.h"
#include "steering.h"
#include "traxxas_remctl.h"
#include "bumper.h"
#include "default_calib.h"

const int ERR_BLINK_GENERAL = 2;
const int ERR_BLINK_IMU = 3;
const int INITED_BLINK = 2;

const int FR_BUMPER_PIN = 24;
const int FR_BUMPER_NEG_PIN = 12;

const int OAKD_PAN_SERVO_PIN = 11;
const int OAKD_TILT_SERVO_PIN = 10;

const int OAKD_PAN_SERVO_HOME = 90;
const int OAKD_TILT_SERVO_HOME = 90;

#define DRIVER_CONTROL
#define PUBLISH_ODOM
#define FAKE_ODOM
//#define PUBLISH_MAG

#define RCCHECKLOG(fn)                        \
    {                                         \
        rcl_ret_t temp_rc = fn;                \
        if ((temp_rc != RCL_RET_OK))           \
        {                                     \
            Logger::log_message(Logger::LogLevel::Error, "Failed to execute function %s, error = %d.", #fn, temp_rc); \
        }                                     \
    }

#define RCCHECK(fn)                          \
    {                                        \
        rcl_ret_t temp_rc = fn;              \
        if ((temp_rc != RCL_RET_OK))         \
        {                                    \
            rclErrorLoop(ERR_BLINK_GENERAL); \
        }                                    \
    }
#define RCCHECK_WITH_BLINK_CODE(blink_code, fn) \
    {                                           \
        rcl_ret_t temp_rc = fn;                 \
        if ((temp_rc != RCL_RET_OK))            \
        {                                       \
            rclErrorLoop(blink_code);           \
        }                                       \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

enum class DriverControlMode { Manual, Auto };

#ifdef PUBLISH_ODOM
    rcl_publisher_t odom_publisher;
    nav_msgs__msg__Odometry odom_msg;
    unsigned long prev_odom_update = 0;
#endif

rcl_publisher_t imu_publisher;
#ifdef PUBLISH_MAG
    rcl_publisher_t mag_publisher;
#endif
rcl_publisher_t bumper_publisher;

rcl_subscription_t twist_subscriber;

#if defined(DRIVER_CONTROL)
rcl_subscription_t driver_control_subscriber;

std_msgs__msg__Int8 driver_control_msg;
#endif

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;

rcl_subscription_t oakd_pan_subscriber;
std_msgs__msg__Int8 oakd_pan_msg;
rcl_subscription_t oakd_tilt_subscriber;
std_msgs__msg__Int8 oakd_tilt_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t imu_timer;
rcl_timer_t bumper_timer;

// add a service called calibrateMag
rcl_service_t calibrate_mag_service;
linorobot2_interfaces__srv__CalibrateMag_Request calibrate_mag_req;
linorobot2_interfaces__srv__CalibrateMag_Response calibrate_mag_res;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
bool micro_ros_init_successful = false;

DriverControlMode driver_control_mode = DriverControlMode::Auto;

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#if defined(PUBLISH_ODOM) && !defined(FAKE_ODOM)
    Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
#endif

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
// Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Steering motor
Motor motor_str_controller(PWM_FREQUENCY, PWM_BITS, MOTOR_STR_INV, MOTOR_STR_PWM, MOTOR_STR_IN_A, MOTOR_STR_IN_B);

const float MAX_TURN_PWM_CORRECTION = 18.0f;

Steering steering(STEERING_FULL_RANGE_DEG, motor_str_controller);

Traxxas_RemCtl traxxas_remote(true, true);

Bumper fr_bumper("front", FR_BUMPER_PIN, FR_BUMPER_NEG_PIN);

Servo oakd_pan_servo;
Servo oakd_tilt_servo;

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER,
    FR_WHEELS_DISTANCE,
    LR_WHEELS_DISTANCE
);

#ifdef PUBLISH_ODOM
    Odometry odometry;
#endif

IMU imu;

float current_rpm1 = 0.0;
float current_rpm2 = 0.0;
float current_rpm3 = 0.0;
float current_rpm4 = 0.0;

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);

    bool imu_ok = imu.init();
    if(!imu_ok)
    {
        std::string errStr = imu.getErrorStr();
        while(1)
        {
            Serial.printf("Error initializing IMU: %s", errStr.c_str());
            flashLED(ERR_BLINK_IMU);
        }
    }
    imu.calibrateMag(DEFAULT_MAG_BIAS, DEFAULT_MAG_SCALE);
    
    imu.setAccelCalib(ACCEL_SCALE, ACCEL_BIAS);
        
    micro_ros_init_successful = false;

    set_microros_serial_transports(Serial);
    motor1_controller.init();
    motor_str_controller.init();
    traxxas_remote.setup();
    oakd_pan_servo.attach(OAKD_PAN_SERVO_PIN);
    oakd_pan_servo.write(OAKD_PAN_SERVO_HOME);
    oakd_tilt_servo.attach(OAKD_TILT_SERVO_PIN);
    oakd_tilt_servo.write(OAKD_TILT_SERVO_HOME);
    flashLED(INITED_BLINK);
}

void loop() {
    switch (state) 
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) 
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) 
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}

void bumperCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
        fr_bumper.publish();
    }
}

void imuCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
        publishImu();
    }
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {        
       moveBase();
       publishOdom();
    }
}

void twistCallback(const void * msgin) 
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

#if defined(DRIVER_CONTROL)
void setDriverControlMode(DriverControlMode mode)
{
    if (driver_control_mode != mode)
    {
        driver_control_mode = mode;
        Logger::log_message(Logger::LogLevel::Info, "Driver control mode changed to %s.", 
                            mode == DriverControlMode::Manual ? "Manual" : "Auto");
        if (mode == DriverControlMode::Manual)
        {
            steering.enable_steering_wheel();
        }
        else
        {
            steering.enable_external_control();
        }
    }        
}

DriverControlMode ConvertDriverControlMsgToEnum(uint8_t mode)
{
    switch (mode)
    {
        case 0: return DriverControlMode::Manual;
        case 1: return DriverControlMode::Auto;
    }
    return DriverControlMode::Manual;
}

void driverControlCallback(const void *msgin)
{
    auto mode = ConvertDriverControlMsgToEnum(driver_control_msg.data);
    if (driver_control_mode != mode)
    {
        setDriverControlMode(mode);
    }
}
#endif

void oakdPanCallback(const void *msgin)
{
    int pan_angle = oakd_pan_msg.data;
    std::clamp(pan_angle, -60, 60);
    oakd_pan_servo.write(OAKD_PAN_SERVO_HOME + pan_angle);
}

void oakdTiltCallback(const void *msgin)
{
    int tilt_angle = oakd_tilt_msg.data;
    std::clamp(tilt_angle, -50, 20);

    oakd_tilt_servo.write(OAKD_TILT_SERVO_HOME + tilt_angle);
}

void moveCallback()
{
    EXECUTE_EVERY_N_MS(20, 
        traxxas_remote.update();
        motor1_controller.spin(traxxas_remote.throttle_pwm()); 
        steering.set_position(traxxas_remote.steering_pwm());
        steering.update(true);
//        Logger::log_message(Logger::LogLevel::Debug, "movecallback Throttle: %d, Steering: %d", traxxas_remote.throttle_pwm(), traxxas_remote.steering_pwm());
    );
}

void calibrateMagCallback(const void *request, void *response)
{
    linorobot2_interfaces__srv__CalibrateMag_Request* req_in = 
    (linorobot2_interfaces__srv__CalibrateMag_Request *)request;
    linorobot2_interfaces__srv__CalibrateMag_Response* res_in = 
    (linorobot2_interfaces__srv__CalibrateMag_Response *) response;
    
    imu.calibrateMag(req_in, res_in, moveCallback);
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    
    // PUBLISHERS

#ifdef PUBLISH_ODOM
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));
#endif

    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));
#ifdef PUBLISH_MAG
    RCCHECK(rclc_publisher_init_default(
        &mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "imu/mag"
    ));
#endif
    // create bumper publisher
    RCCHECK(fr_bumper.create(node));

    Logger::create_logger(node);   

    // SUBSCRIPTIONS

#if defined(DRIVER_CONTROL)
    // subscribe to driver control mode
    RCCHECK(rclc_subscription_init_default(
        &driver_control_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "driver_control_mode"));
#endif

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));

    // create oakd pan subscriber
    RCCHECK(rclc_subscription_init_default( 
        &oakd_pan_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "oakd/pan"
    ));

    // create oakd tilt subscriber
    RCCHECK(rclc_subscription_init_default( 
        &oakd_tilt_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "oakd/tilt"
    ));

    // create timer for publishing bumper at 10 Hz (1000/100)
    const unsigned int bumper_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &bumper_timer, 
        &support,
        RCL_MS_TO_NS(bumper_timeout),
        bumperCallback
    ));

    // create timer for publishing IMU at 10 Hz (1000/100)
    const unsigned int imu_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &imu_timer, 
        &support,
        RCL_MS_TO_NS(imu_timeout),
        imuCallback
    ));

    // create timer for actuating the motors at 25 Hz (1000/40)
    const unsigned int control_timeout = 40;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));

    // SERVICES
    calibrate_mag_service = rcl_get_zero_initialized_service();
    RCCHECK(rclc_service_init_default(
        &calibrate_mag_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(linorobot2_interfaces, srv, CalibrateMag),
        "calibrate_mag"));

    executor = rclc_executor_get_zero_initialized_executor();
    
    // 4 subscribers + 1 service + 3 timer
    const unsigned int executor_handles = 4 + 1 + 3;
    RCCHECK(rclc_executor_init(&executor, &support.context, executor_handles, & allocator));

    // The add order maters, the first added is the first to be run
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &bumper_timer));

#if defined(DRIVER_CONTROL)
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &driver_control_subscriber,
        &driver_control_msg,
        &driverControlCallback,
        ON_NEW_DATA));
#endif

    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &oakd_pan_subscriber, 
        &oakd_pan_msg,
        &oakdPanCallback, 
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &oakd_tilt_subscriber, 
        &oakd_tilt_msg, 
        &oakdTiltCallback, 
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_service(&executor, &calibrate_mag_service, 
        &calibrate_mag_req, &calibrate_mag_res, calibrateMagCallback));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);
    micro_ros_init_successful = true;
    return true;
}

bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    fr_bumper.destroy(node);

#ifdef PUBLISH_ODOM
    rcl_publisher_fini(&odom_publisher, &node);
#endif
    rcl_publisher_fini(&imu_publisher, &node);
#ifdef PUBLISH_MAG
    rcl_publisher_fini(&mag_publisher, &node);
#endif
    rcl_publisher_fini(&bumper_publisher, &node);

#if defined(DRIVER_CONTROL)
    rcl_subscription_fini(&driver_control_subscriber, &node);
#endif

    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_subscription_fini(&oakd_pan_subscriber, &node);
    rcl_subscription_fini(&oakd_tilt_subscriber, &node);
    rcl_service_fini(&calibrate_mag_service, &node);

    rcl_node_fini(&node);
    rcl_timer_fini(&bumper_timer);
    rcl_timer_fini(&imu_timer);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, LOW);
    
    micro_ros_init_successful = false;

    return true;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    // motor2_controller.brake();
    // motor3_controller.brake();
    // motor4_controller.brake();
}

float steer(float steering_angle)
{
    float angle_deg = -steering_angle * 180.0 / M_PI;
    angle_deg = steering.set_position_deg(angle_deg);
    return -angle_deg * M_PI / 180.0;
}

float getSteeringPos()
{
    return -steering.get_position_deg() * M_PI / 180.0;
}

bool directionChange(float cur_rpm, float req_rpm)
{
    return abs(cur_rpm) > 1.0 && sgnf(cur_rpm) != sgnf(req_rpm);
}


// For converting twist msg to Ackermann x vel and steering angle
// See convertTransRotVelToSteeringAngle() in 
// https://github.com/rst-tu-dortmund/teb_local_planner/blob/melodic-devel/src/teb_local_planner_ros.cpp
float rotational_vel_to_steering_angle(float x_vel, float w_vel, float wheelbase)
{
    if (x_vel == 0.0 || w_vel == 0.0)
    {
        return 0.0;
    }
    float radius = x_vel / w_vel;

    if (fabs(radius) < STEERING_MIN_TURN_RADIUS)
    {
        radius = STEERING_MIN_TURN_RADIUS * sgnf(radius);
    }

    return atan(wheelbase / radius);
}


// RPM to PWM conversion
// This is a quadratic fit based on human measured data on the Traxxas TRX-4 Sport wheel rotations / s
int rpm2pwm(float rpm, float steering_angle)
{
    int pwm = 0;

    if (rpm > 0.0)
    {
        pwm = 68.02266 + 0.4708083 * rpm + 0.0002057336 * rpm * rpm;
    }
    else if (rpm < 0.0)
    {
        pwm = 0.4122096 * rpm - 64.01235;
    }

    if (steering_angle != 0.0)
    {
        pwm += MAX_TURN_PWM_CORRECTION * sgnf(rpm) * fabs(steering_angle) / STEERING_HALF_RANGE_RAD;
    }

    return pwm;
}

float pwm2rpm(int pwm, float steering_angle)
{
    float rpm = 0.0;

    if (steering_angle != 0.0)
    {
        if (pwm != 0) 
        {
            int adj = MAX_TURN_PWM_CORRECTION * sgni(pwm) * fabs(steering_angle) / STEERING_HALF_RANGE_RAD;
            if (abs(adj) < abs(pwm))
            {
                pwm -= adj;
            }
        }
    }

    float root1 = 0.0;
    float root2 = 0.0;

    if (pwm > 0)
    {
        float a = 0.0002057336;
        float b = 0.4708083;
        float c = 68.02266 - pwm;

        float discriminant = b*b - 4*a*c;

        if (discriminant < 0) {
            // No real roots
            return 0;
        }

        root1 = (-b + sqrt(discriminant)) / (2*a);
        root2 = (-b - sqrt(discriminant)) / (2*a);

    }
    else if (pwm < 0)
    {
        rpm = fmin(0.0, (pwm + 64.01235) / 0.4122096);
    }

    // Choose the root that has the same sign as pwm
    if (pwm > 0)
    {
        rpm = fmax(0.0, fmax(root1, root2));
    }

    return rpm;
}

void moveBase()
{
#ifdef PUBLISH_ODOM
    #ifdef FAKE_ODOM
        static float prev_motor1_rpm = 0.0;
        current_rpm1 = prev_motor1_rpm;
    #else
        // get the current speed of each motor
        current_rpm1 = motor1_encoder.getRPM();
    #endif
#endif

    float steering_angle = 0.0;

    traxxas_remote.update();

    bool estop = traxxas_remote.estop_asserted() && driver_control_mode == DriverControlMode::Auto;

#if defined(DRIVER_CONTROL)
    // Handle Manual drive mode from Traxxas Remote
    if (driver_control_mode == DriverControlMode::Manual)
    {
        int throttle_pwm = traxxas_remote.throttle_pwm();
        int steering_pwm = traxxas_remote.steering_pwm();
        motor1_controller.spin(throttle_pwm);
        steering.set_position(steering_pwm);

        #ifdef FAKE_ODOM
            // prev_motor1_rpm = pwm2rpm(throttle_pwm, steering.get_position_deg() * M_PI / 180.0);
            // EXECUTE_EVERY_N_MS(240, Logger::log_message(Logger::LogLevel::Debug, "Throttle: %d, Steering: %d%", 
            //     throttle_pwm, steering_pwm););
        #endif
    }
    else // Auto mode or disabled
#endif
    {
        float speed_x = 0.0;
        float steering_angle = 0.0;

        // Handle twist msg input (driven by the twist telop or a
        // navigation controller)

        // brake if there's no command received, or when it's only the first command sent
        if(((millis() - prev_cmd_time) >= 400)) 
        {
            digitalWrite(LED_PIN, HIGH);
        }
        else
        {
            speed_x = twist_msg.linear.x;

            // Calculate steering angle from x velocity, twist and wheelbase
            // http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
            steering_angle = rotational_vel_to_steering_angle(twist_msg.linear.x, twist_msg.angular.z, FR_WHEELS_DISTANCE);

            //EXECUTE_EVERY_N_MS(240, Logger::log_message(Logger::LogLevel::Debug, "Steering angle %f, xvel: %f, zvel: %f",
            //     steering_angle*180.0/M_PI, twist_msg.linear.x, twist_msg.angular.z));
        }

        // get the required rpm for each motor based on required velocities, and base used
        Kinematics::rpm req_rpm = kinematics.getRPM(
            speed_x,
            0,
            steering_angle);

#if defined(PUBLISH_ODOM) && !defined(FAKE_ODOM)
        // Don't drive motor in the opposite direction until it stops
        if (directionChange(current_rpm1, req_rpm.motor1) || estop)
        {
            req_rpm.motor1 = 0.0;
        }
        // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
        // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
        motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
#else
        if (estop)
        {
            req_rpm.motor1 = 0.0;
        }

        int pwm = rpm2pwm(req_rpm.motor1, steering_angle);
        
        // Logger::log_message(Logger::LogLevel::Debug, "req_rpm = %f, pwm = %d",
        //     req_rpm.motor1, pwm);
    
        motor1_controller.spin(pwm);

    #ifdef FAKE_ODOM
        prev_motor1_rpm = req_rpm.motor1;
        {
    #endif        
#endif

        if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
            if (speed_x != 0.0) // don't change steering when robot is not moving because angle is always 0
                steer(steering_angle);
        }        
    }

#ifdef PUBLISH_ODOM
    Kinematics::velocities current_vel;
    if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
    {
        current_vel = kinematics.getVelocities(getSteeringPos(), current_rpm1);
    }
    else
    {
        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    }

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
#endif

    steering.update(!estop);
}

void computeRollPitch(const geometry_msgs__msg__Vector3& linear_accel, double& roll, double& pitch)
{
    roll = atan2(linear_accel.y, linear_accel.z);
    pitch = atan2(linear_accel.x, sqrt(linear_accel.z * linear_accel.z + linear_accel.y * linear_accel.y));
}

void computeYaw(double roll, double pitch, const sensor_msgs__msg__MagneticField& mag, double& yaw)
{
    double magLength = sqrt(mag.magnetic_field.x * mag.magnetic_field.x + 
                            mag.magnetic_field.y * mag.magnetic_field.y + 
                            mag.magnetic_field.z * mag.magnetic_field.z);
    
    // Check for zero magnitude to avoid division by zero
    if (magLength == 0) {
        yaw = 0; // or some error value
        return;
    }
    
    double normMagVals[3] = {mag.magnetic_field.x / magLength, 
                             mag.magnetic_field.y / magLength, 
                             mag.magnetic_field.z / magLength};
    
    
    yaw = atan2(sin(roll) * normMagVals[2] - cos(roll) * normMagVals[1],
                cos(pitch) * normMagVals[0] + sin(roll) * sin(pitch) * normMagVals[1] + cos(roll) * sin(pitch) * normMagVals[2]);

    // Add 90 degrees to yaw to align with ROS convention ENU, East = 0 degrees
    yaw += M_PI / 2;  // Add 90 degrees in radians

    // Normalize yaw to [-π, π)
    if (yaw > M_PI) {
        yaw -= 2 * M_PI;
    } else if (yaw < -M_PI) {
        yaw += 2 * M_PI;
    }                
}

void convertRollPitchYawToQuaternion(double roll, double pitch, double yaw, sensor_msgs__msg__Imu& imu_msg)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    imu_msg.orientation.w = cy * cp * cr + sy * sp * sr;
    imu_msg.orientation.x = cy * cp * sr - sy * sp * cr;
    imu_msg.orientation.y = sy * cp * sr + cy * sp * cr;
    imu_msg.orientation.z = sy * cp * cr - cy * sp * sr;

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[8] = 0.0025;
}

void publishImu()
{
    imu_msg = imu.getData();
    mag_msg = imu.getMagneticField();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    computeRollPitch(imu_msg.linear_acceleration, roll, pitch);
    computeYaw(roll, pitch, mag_msg, yaw);
    convertRollPitchYawToQuaternion(roll, pitch, yaw, imu_msg); 

    struct timespec time_stamp = getTime();
    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
#ifdef PUBLISH_MAG
    RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
#endif
}

#ifdef PUBLISH_ODOM
void publishOdom()
{
    odom_msg = odometry.getData();
    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL)); 
}
#endif

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop(int n_times) 
{
    while(true)
    {
        flashLED(n_times);
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}