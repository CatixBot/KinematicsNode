#include "servo/ServoPublisher.h"

#include <catix_messages/ServoState.h>

servo::ServoPublisher::ServoPublisher(size_t servoIndex, ros::NodeHandle& node)
    : servoIndex(servoIndex)
    , publisherServoState(node.advertise<catix_messages::ServoState>("Catix/Servo", 1))
{
}

bool servo::ServoPublisher::setAngle(double servoAngle)
{
    catix_messages::ServoState servoStateMessage;
    servoStateMessage.servo_index = static_cast<uint8_t>(this->servoIndex);
    servoStateMessage.rotate_angle = servoAngle;
    this->publisherServoState.publish(servoStateMessage);
    
    ROS_INFO("Joint %d: [%frad]", this->servoIndex, servoAngle);
    return true;
}
