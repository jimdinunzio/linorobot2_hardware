#include <Arduino.h>
#include <rclc/rclc.h>

#include "bumper.h"

Bumper::Bumper(const String &name, int pin, int neg_pin):
    name_(name),
    pin_(pin),
    neg_pin_(neg_pin),
    inited_(false)
{
}

rcl_ret_t Bumper::create(rcl_node_t &node)
{
    if (inited_) {
        return RMW_RET_OK;
    }

    String topic_base = String("bumper/" + name_);

    rcl_ret_t ret_val = rclc_publisher_init_default( 
        &bumper_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        String(topic_base + "/state").c_str()
    );

    if (ret_val != RMW_RET_OK) {
        return ret_val;
    }

    pinMode(pin_, INPUT);

    if (neg_pin_ != -1) {
        pinMode(neg_pin_, OUTPUT);
        digitalWrite(neg_pin_, LOW);
    }

    inited_ = true;
    return ret_val;
}

void Bumper::destroy(rcl_node_t &node)
{
    if (!inited_) {
        return;
    }
    inited_ = false;
    rcl_publisher_fini(&bumper_publisher_, &node);
}

void Bumper::publish()
{
    if (!inited_) {
        return;
    }

    bumper_msg.data = digitalRead(pin_);
    rcl_publish(&bumper_publisher_, &bumper_msg, NULL);
}