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
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>

#include "config.h"
#include "logger.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "util.h"
#include "steering.h"
#include "traxxas_remctl.h"
#include "bumper.h"

const int ERR_BLINK_GENERAL = 2;
const int ERR_BLINK_IMU = 3;

const int FR_BUMPER_PIN = 24;
const int FR_BUMPER_NEG_PIN = 12;

const int OAKD_PAN_SERVO_PIN = 11;
const int OAKD_TILT_SERVO_PIN = 10;

const int OAKD_PAN_SERVO_HOME = 90;
const int OAKD_TILT_SERVO_HOME = 90;

#define DRIVER_CONTROL
//#define PUBLISH_ODOM

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
rcl_publisher_t bumper_publisher;

rcl_subscription_t twist_subscriber;

#if defined(DRIVER_CONTROL)
rcl_subscription_t driver_control_subscriber;

std_msgs__msg__Int8 driver_control_msg;
#endif

sensor_msgs__msg__Imu imu_msg;
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

// Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
// Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Steering motor
Motor motor_str_controller(PWM_FREQUENCY, PWM_BITS, MOTOR_STR_INV, MOTOR_STR_PWM, MOTOR_STR_IN_A, MOTOR_STR_IN_B);

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

    bool imu_ok = imu.init();
    if(!imu_ok)
    {
        while(1)
        {
            flashLED(3);
        }
    }
    
    micro_ros_init_successful = false;

    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    motor1_controller.init();
    motor_str_controller.init();
    traxxas_remote.setup();
    oakd_pan_servo.attach(OAKD_PAN_SERVO_PIN);
    oakd_pan_servo.write(OAKD_PAN_SERVO_HOME);
    oakd_tilt_servo.attach(OAKD_TILT_SERVO_PIN);
    oakd_tilt_servo.write(OAKD_TILT_SERVO_HOME);
    flashLED(2);
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

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {        
       moveBase();
       publishData();
       fr_bumper.publish();
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
    // TIMER

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));

    executor = rclc_executor_get_zero_initialized_executor();
    
    // 5 handles = 4 subscribers + 1 timer
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, & allocator));

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

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

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

#if defined(DRIVER_CONTROL)
    rcl_subscription_fini(&driver_control_subscriber, &node);
#endif

    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_subscription_fini(&oakd_pan_subscriber, &node);
    rcl_subscription_fini(&oakd_tilt_subscriber, &node);

    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);
    
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
    return abs(cur_rpm) > 1.0 && sgn(cur_rpm) != sgn(req_rpm);
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
        radius = STEERING_MIN_TURN_RADIUS * sgn(radius);
    }

    return atan(wheelbase / radius);
}

void moveBase()
{
#ifdef PUBLISH_ODOM
    // get the current speed of each motor
    current_rpm1 = motor1_encoder.getRPM();
#endif

    float steering_angle = 0.0;

    traxxas_remote.update();

    bool estop = traxxas_remote.estop_asserted() && driver_control_mode == DriverControlMode::Auto;

#if defined(DRIVER_CONTROL)
    // Handle Manual drive mode from Traxxas Remote
    if (driver_control_mode == DriverControlMode::Manual)
    {
        motor1_controller.spin(traxxas_remote.throttle_pwm());
        steering.set_position(traxxas_remote.steering_pwm());
        //EXECUTE_EVERY_N_MS(240, Logger::log_message(Logger::LogLevel::Debug, "Throttle: %d%, Steering: %d%", 
        //    traxxas_remote.throttle_percent(), traxxas_remote.steering_percent()););
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
            Logger::log_message(Logger::LogLevel::Debug, "Steering angle %f, xve: %f, zvel: %f",
                steering_angle*180.0/M_PI, twist_msg.linear.x, twist_msg.angular.z);
        }
        // get the required rpm for each motor based on required velocities, and base used
        Kinematics::rpm req_rpm = kinematics.getRPM(
            speed_x,
            0,
            steering_angle);

#ifdef PUBLISH_ODOM
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

        motor1_controller.spin(req_rpm.motor1);
#endif

        if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
        {
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

void publishData()
{
#ifdef PUBLISH_ODOM
    odom_msg = odometry.getData();
#endif

    imu_msg = imu.getData();

    struct timespec time_stamp = getTime();

#ifdef PUBLISH_ODOM
    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
#endif

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
#ifdef PUBLISH_ODOM
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
#endif
}

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