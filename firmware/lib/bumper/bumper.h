#ifndef BUMPER_H
#define BUMPER_H

#include <Arduino.h>

#include <rcl/rcl.h>
#include <std_msgs/msg/bool.h>

class Bumper
{
    public:
        Bumper(const String &name, int pin, int neg_pin = -1);
        rcl_ret_t create(rcl_node_t &node);
        void destroy(rcl_node_t &node);
        void publish();

    private:
        String name_;
        int pin_;
        int neg_pin_;
        
        bool inited_;

        rcl_publisher_t bumper_publisher_;
        
        std_msgs__msg__Bool bumper_msg;
};

#endif // BUMPER_H