#include "servo/ServoPublisher.h"

#include <ros/ros.h>

servo::ServoPublisher::ServoPublisher(size_t servoIndex)
    : servoIndex(servoIndex)
{
}

bool servo::ServoPublisher::setAngle(double angleRadians)
{
    ROS_INFO("Servo %d: %frad", this->servoIndex, angleRadians);

    return true;
}
