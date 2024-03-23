#include <Arduino.h>
#include "traxxas_remctl.h"

// These variables are for manual mode
// holds the update flags defined above
static volatile uint8_t bUpdateFlagsShared = 0;
// shared variables are updated by the ISR and read by loop.
static volatile uint16_t unThrottleInShared = 0;
static volatile uint16_t unSteeringInShared = 0;
// To record the rising edge of a pulse in the calcInput functions
static uint32_t ulThrottleStart = 0;
static uint32_t ulSteeringStart = 0;

// Assign your channel in pins
#define THROTTLE_IN_ESTOP_PIN 3
#define STEERING_IN_PIN 4

// channels have new signals flags
#define THROTTLE_ESTOP_FLAG 1
#define STEERING_FLAG 2

#define MIN_PWM_THROTTLE 1000
#define MAX_PWM_THROTTLE 2000
#define MIN_PWM_STEERING 1000
#define MAX_PWM_STEERING 2000
#define ESTOP_PWM_THRESHOLD 1700

Traxxas_RemCtl::Traxxas_RemCtl(bool throttle_enabled, bool steering_enabled)
: throttle_enabled_(throttle_enabled), steering_enabled_(steering_enabled),
  throttle_pwm_(0), steering_pwm_(0)
{
}

void Traxxas_RemCtl::setup()
{
    pinMode(THROTTLE_IN_ESTOP_PIN, INPUT);
    pinMode(STEERING_IN_PIN, INPUT);

    enable_steering_channel(steering_enabled_);
    enable_throttle_channel(throttle_enabled_);
}

void Traxxas_RemCtl::enable_steering_channel(bool enable)
{
    if (enable)
    {
        attachInterrupt(digitalPinToInterrupt(STEERING_IN_PIN), calc_steering_isr, CHANGE);
    }
    else
    {
        detachInterrupt(digitalPinToInterrupt(STEERING_IN_PIN));
        unSteeringInShared = 0;
        ulSteeringStart = 0;
        bUpdateFlagsShared = 0;
    }
    steering_enabled_ = enable;
}

void Traxxas_RemCtl::enable_throttle_channel(bool enable)
{
    if (enable)
    {
        attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_ESTOP_PIN), calc_throttle_isr, CHANGE);
    }
    else
    {
        detachInterrupt(digitalPinToInterrupt(THROTTLE_IN_ESTOP_PIN));
        unSteeringInShared = 0;
        ulThrottleStart = 0;
        bUpdateFlagsShared = 0;
    }
    throttle_enabled_ = enable;
}

bool Traxxas_RemCtl::estop_asserted() const
{
  return throttle_pwm_ <= ESTOP_PWM_THRESHOLD;
}

// simple interrupt service routine
void Traxxas_RemCtl::calc_throttle_isr()
{
  uint16_t tempThrottle;
  // if the pin is high, its a rising edge
  if(digitalRead(THROTTLE_IN_ESTOP_PIN) == HIGH)
  { 
    ulThrottleStart = micros();
  }
  else
  {
    tempThrottle = (uint16_t)(micros() - ulThrottleStart);
    if (tempThrottle >= MIN_PWM_THROTTLE && tempThrottle <= MAX_PWM_THROTTLE)
    {
      unThrottleInShared = tempThrottle;
      bUpdateFlagsShared |= THROTTLE_ESTOP_FLAG;
    }
  }
}

void Traxxas_RemCtl::calc_steering_isr()
{
  uint16_t tempSteering;
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  { 
    ulSteeringStart = micros();
  }
  else
  {
    tempSteering = (uint16_t)(micros() - ulSteeringStart);
    if (tempSteering >= MIN_PWM_STEERING && tempSteering <= MAX_PWM_STEERING)
    {
      unSteeringInShared = tempSteering;
      bUpdateFlagsShared |= STEERING_FLAG;
    }
  }
}

int Traxxas_RemCtl::throttle_percent()
{
    return map(throttle_pwm_, MIN_PWM_THROTTLE, MAX_PWM_THROTTLE, -100, 100);
}

int Traxxas_RemCtl::steering_percent()
{
    return map(steering_pwm_, MIN_PWM_STEERING, MAX_PWM_STEERING, -100, 100);
}

void Traxxas_RemCtl::update()
{
    uint8_t bUpdateFlags = 0;
    uint16_t newThrottle = 0;
    uint16_t newSteering = 0;

    // check if any channels have a new signal
    if(bUpdateFlagsShared)
    {
        noInterrupts(); // turn interrupts off 
        bUpdateFlags = bUpdateFlagsShared;
        
        if(bUpdateFlags & THROTTLE_ESTOP_FLAG)
        {
            throttle_pwm_ = unThrottleInShared;
        }
        
        if(bUpdateFlags & STEERING_FLAG)
        {
            steering_pwm_ = unSteeringInShared;
        }
        
        bUpdateFlagsShared = 0;
        interrupts(); 
    }
}