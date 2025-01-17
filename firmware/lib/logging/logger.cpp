#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float32.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "logger.h"

namespace {
const int MAX_MSG_LEN = 1000;    
Logger *logger = nullptr;
}

bool Logger::create_logger(rcl_node_t &node)
{
    if (logger) {
        return false;
    }

    logger = new Logger();
    if (logger->create(node)) {
        return true;
    }
    delete logger;
    logger = nullptr;
    return false;
}

bool Logger::destroy_logger(rcl_node_t &node)
{
    if (!logger) {
        return false;
    }
    logger->destroy(node);
    delete logger;
    logger = nullptr;
    return true;
}

void Logger::log_message(LogLevel level, const char * fmt, ...)
{
    if (!logger) {
        return;
    }
	char buf[MAX_MSG_LEN];
    va_list args;
    va_start(args, fmt);

    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    logger->log(level, buf);
}

Logger::Logger():
    inited_(false)
{
}

bool Logger::create(rcl_node_t &node)
{
    if (inited_) {
        return false;
    }

    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
    publisher_options.qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    if (rclc_publisher_init(
        &publisher_log_,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
        "rosout",
        &publisher_options.qos) != RCL_RET_OK) {

        return false;
    }

    inited_ = true;
    return true;
}

void Logger::destroy(rcl_node_t &node)
{
    if (!inited_) {
        return;
    }
    inited_ = false;
    rcl_publisher_fini(&publisher_log_, &node);
}

void Logger::log(Logger::LogLevel level, const char *msg)
{
    if (!inited_) {
        return;
    }

    switch(level) {
        case LogLevel::Error:
            log_msg.level = rcl_interfaces__msg__Log__ERROR;
            break;
        case LogLevel::Warn:
            log_msg.level = rcl_interfaces__msg__Log__WARN;
            break;
        case LogLevel::Debug:
            log_msg.level = rcl_interfaces__msg__Log__DEBUG;
            break;
        default:    
            log_msg.level = rcl_interfaces__msg__Log__INFO;
            break;
    }

    log_msg.name.data = (char*)"linorobot_truckasaurus";
    log_msg.name.size = strlen(log_msg.name.data) + 1;
    log_msg.msg.data = (char*)msg;
    log_msg.msg.size = strlen(msg) + 1;
    log_msg.file.data = (char*)"";
    log_msg.file.size = strlen(log_msg.file.data) + 1;
    log_msg.function.data = (char*)"";
    log_msg.function.size = strlen(log_msg.function.data) + 1;
    log_msg.line = 0;
    rcl_publish(&publisher_log_, &log_msg, NULL);
}
