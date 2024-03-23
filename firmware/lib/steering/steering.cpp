
#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "config.h"
#include "steering.h"

#undef DEBUG_PRINTS

namespace
{
    const long DEF_UPDATE_PERIOD_MS = 50;
}

Steering::Steering(uint16_t full_range_deg, MotorInterface &motor) 
    : limit_left_(PWM_MIN),
    limit_right_(PWM_MAX),
    pwm_per_deg_((float)(PWM_MAX - PWM_MIN)/(float)full_range_deg),
    motor_(motor),
    main_state_(State::kExternal),
    target_pwm_(0),
    pwm_(0),
    next_update_(0)
{
}


Steering::State Steering::get_state() const
{
    return main_state_;
}

bool Steering::disable()
{
    motor_.spin(0);
    main_state_ = State::kDisabled;
    return true;
}

bool Steering::enable_steering_wheel()
{
    motor_.spin(0);
    main_state_ = State::kWheel;
    return true;
}

bool Steering::enable_external_control()
{
    motor_.spin(0);
    main_state_ = State::kExternal;
    return true;
}

int16_t Steering::set_position(int16_t target_pwm)
{
    if (main_state_ == State::kExternal)
    {
        if (target_pwm >= limit_left_ && target_pwm <= limit_right_)
        {
            target_pwm_ = target_pwm;
        }
    }
    else // main_state_ == State::kWheel
    {
        target_pwm_ = target_pwm;
    }
    return target_pwm_;
}

float Steering::set_position_deg(float target_pos_deg)
{
    int16_t target_pwm = target_pos_deg * pwm_per_deg_;
    if (main_state_ == State::kExternal) {
        if (target_pwm < limit_left_) {
            target_pwm = limit_left_;
        } else if (target_pwm > limit_right_) {
            target_pwm = limit_right_;
        }            
    }
    target_pwm_ = target_pwm;
    return target_pwm_ / pwm_per_deg_;
}

long Steering::apply_position()
{
    if (target_pwm_ < limit_left_)
    {
        target_pwm_ = limit_left_;
    }
    else if (target_pwm_ > limit_right_)
    {
        target_pwm_ = limit_right_;
    }

    //Serial.print("new target_pwm_ = ");
    //Serial.println(target_pwm_);
    motor_.spin(target_pwm_);
    pwm_ = target_pwm_;

    return DEF_UPDATE_PERIOD_MS;
}

long Steering::steering_wheel_update()
{
    if (target_pwm_ != pwm_)
        return apply_position();
    else
        return DEF_UPDATE_PERIOD_MS;
}

long Steering::external_control_update()
{
    // pos set via method
    if (target_pwm_ != pwm_)
        return apply_position();
    else
        return DEF_UPDATE_PERIOD_MS;
}

void Steering::update(bool enable)
{
    unsigned long now = millis();

    if ((long)(now - next_update_) < 0)
    {
        return;
    }

    unsigned long next_update_delta = DEF_UPDATE_PERIOD_MS;

    // Stop control if steering is disabled.
    // This is used such when emergency stop is enabled.  In that case
    // the drive power will be disabled, so the control loop of this
    // object should go idle and not drive the loop harder to avoid
    // hammering the motor when estop is de-asserted.
    if (!enable) {
        motor_.spin(0);
        return;
    }

    switch (main_state_)
    {
    default:
        break;

    case State::kDisabled:
        break;

    case State::kWheel:
        next_update_delta = steering_wheel_update();
        break;

    case State::kExternal:
        next_update_delta = external_control_update();
        break;
    }
    next_update_ = now + next_update_delta;
}
