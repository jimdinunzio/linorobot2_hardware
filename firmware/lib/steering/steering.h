#ifndef STEERING_H
#define STEERING_H

#include "Arduino.h"
#include "motor.h"
#include "pid.h"

class Steering
{
public:
    enum class State
    {
        kWheel,
        kExternal,
        kDisabled,
    };

public:
    Steering(uint16_t full_range_deg, MotorInterface &motor);

    State get_state() const;

    bool disable();
    bool enable_steering_wheel();
    bool enable_external_control();

    int16_t get_position() const { return target_pwm_; }
    int16_t set_position(int16_t target_pwm);
    
    float get_position_deg() const { return (float)target_pwm_ / (target_pwm_ > Motor::PWM_NEUTRAL ? pwm_per_deg_right_: pwm_per_deg_left_); }
    float set_position_deg(float target_pos_deg);

    void update(bool enable);
    
private:
    long steering_wheel_update();
    long external_control_update();

    long apply_position();
private:
    int16_t limit_left_;
    int16_t limit_right_;
    float pwm_per_deg_right_;
    float pwm_per_deg_left_;
    float wheel_scale_factor_;
    
    MotorInterface &motor_;

    State main_state_;

    int16_t target_pwm_;
    int16_t pwm_;
    unsigned long next_update_;
};

#endif // STEERING_H
