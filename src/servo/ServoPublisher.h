#pragma once

#include <ros/ros.h>

#include "servo/ServoInterface.h"

namespace servo
{
	class ServoPublisher : public servo::IServo
	{
	public:
		ServoPublisher(size_t servoIndex, ros::NodeHandle &node);

	public:
		bool setAngle(double angleRadians) override;

	private:
		size_t servoIndex;
		ros::Publisher publisherServoState;
	};
}
