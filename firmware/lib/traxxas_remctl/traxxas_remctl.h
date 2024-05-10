
#include <vector>

class Traxxas_RemCtl
{
    public:

        Traxxas_RemCtl(bool throttle_enabled, bool steering_enabled);

        void setup();

        void update();

        int throttle_pwm() const { return throttle_pwm_ - PWM_NEUTRAL; } 
        int steering_pwm() const { return steering_pwm_ - PWM_NEUTRAL; }
        int throttle_percent();
        int steering_percent();

        bool estop_asserted() const;

        void enable_steering_channel(bool enable);
        void enable_throttle_channel(bool enable);

        bool throttle_enabled() const { return throttle_enabled_; }
        bool steering_enabled() const { return steering_enabled_; }

    private:

        static const int PWM_NEUTRAL = 1500;
                
        static void calc_throttle_isr();
        static void calc_steering_isr();

        bool throttle_enabled_;
        bool steering_enabled_;
        int throttle_pwm_;
        int steering_pwm_; 
};