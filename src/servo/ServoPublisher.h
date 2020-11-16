#pragma once

#include "servo/ServoInterface.h"

namespace servo
{
	class ServoPublisher : public servo::IServo
	{
	public:
		ServoPublisher(size_t servoIndex);

	public:
		bool setAngle(double angleRadians) override;

	private:
		size_t servoIndex;
	};
}
