#include "CatixKinematics.h"

#include "platform/PlatformParameters.h"
#include "platform/Platform8Dof.h"
#include "limb/LimbSegment.h"
#include "limb/PolarCoordinates.h"
#include "limb/Leg2Dof.h"
#include "servo/ServoPublisher.h"

#include <cmath>
#include <catix_messages/ServoState.h>

//---------------------------------------------------------------------------

#define NUMBER_OF_SERVOS 8

//---------------------------------------------------------------------------

std::vector<std::shared_ptr<servo::IServo>> makeServos(size_t numberOfServos)
{
    std::vector<std::shared_ptr<servo::IServo>> servos;

    for (size_t servoIndex = 0; servoIndex < numberOfServos; ++servoIndex)
    {
        servos.emplace_back(std::make_shared<servo::ServoPublisher>(servoIndex));
    }

    return servos;
}

std::vector<std::shared_ptr<limb::ILimb2Dof>> makeLegs2Dof(
    const std::vector<std::shared_ptr<servo::IServo>>& servos)
{
    const size_t EXPECTED_NUMBER_OF_SERVOS = 8;
    const size_t PROVIDED_NUMBER_OF_SERVOS = servos.size();

    if (PROVIDED_NUMBER_OF_SERVOS != EXPECTED_NUMBER_OF_SERVOS)
    {
        ROS_ERROR("Legs can't be constructed as %d servos provided, but %d servos expected", 
            PROVIDED_NUMBER_OF_SERVOS, EXPECTED_NUMBER_OF_SERVOS);
        return {};
    }

    std::vector<std::shared_ptr<limb::ILimb2Dof>> legs;

    const double LOWER_SEGMENT_LENGTH_METERS = 0.060;
    const double UPPER_SEGMENT_LENGTH_METERS = 0.049;

    limb::LimbSegment frontLeftLowerSegment;
    frontLeftLowerSegment.jointServo = servos[0];
    frontLeftLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment frontLeftUpperSegment;
    frontLeftUpperSegment.jointServo = servos[1];
    frontLeftUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto frontLeftLeg = std::make_shared<limb::Leg2Dof>(frontLeftLowerSegment, frontLeftUpperSegment);
    legs.push_back(frontLeftLeg);

    limb::LimbSegment frontRightLowerSegment;
    frontRightLowerSegment.jointServo = servos[2];
    frontRightLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment frontRightUpperSegment;
    frontRightUpperSegment.jointServo = servos[3];
    frontRightUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto frontRightLeg = std::make_shared<limb::Leg2Dof>(frontRightLowerSegment, frontRightUpperSegment);
    legs.push_back(frontRightLeg);

    limb::LimbSegment rearRightLowerSegment;
    rearRightLowerSegment.jointServo = servos[4];
    rearRightLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment rearRightUpperSegment;
    rearRightUpperSegment.jointServo = servos[5];
    rearRightUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto rearRightLeg = std::make_shared<limb::Leg2Dof>(rearRightLowerSegment, rearRightUpperSegment);
    legs.push_back(rearRightLeg);

    limb::LimbSegment rearLeftLowerSegment;
    rearLeftLowerSegment.jointServo = servos[6];
    rearLeftLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment rearLeftUpperSegment;
    rearLeftUpperSegment.jointServo = servos[7];
    rearLeftUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto rearLeftLeg = std::make_shared<limb::Leg2Dof>(rearLeftLowerSegment, rearLeftUpperSegment);
    legs.push_back(rearLeftLeg);

    return legs;
}

std::unique_ptr<platform::IPlatform> makePlatform8Dof(const std::vector<std::shared_ptr<limb::ILimb2Dof>>& legs)
{
    const size_t EXPECTED_NUMBER_OF_LEGS = 4;
    const size_t PROVIDED_NUMBER_OF_LEGS = legs.size();

    if (PROVIDED_NUMBER_OF_LEGS != EXPECTED_NUMBER_OF_LEGS)
    {
        ROS_ERROR("Platform can't be constructed as %d legs provided, but %d legs expected", 
            PROVIDED_NUMBER_OF_LEGS, EXPECTED_NUMBER_OF_LEGS);
        return nullptr;
    }

    platform::Platform8DofParameters platformParameters;
    platformParameters.frontLeftLeg = legs[0];
    platformParameters.frontRightLeg = legs[1];
    platformParameters.rearRightLeg = legs[2];
    platformParameters.rearLeftLeg = legs[3];

    return std::make_unique<platform::Platform8Dof>(platformParameters);
}

//---------------------------------------------------------------------------

CatixKinematics::CatixKinematics()
{
    this->servos = makeServos(NUMBER_OF_SERVOS);
    publisherServo = node.advertise<catix_messages::ServoState>("Catix/Servos", 1);
    ROS_INFO("Servos publisher ready...");

    this->legs = makeLegs2Dof(this->servos);
    subscriberLeg = node.subscribe("Catix/Leg2Dof", 1, &CatixKinematics::listenerLegState, this);
    ROS_INFO("2DOF legs listener is ready...");
 
    this->platform = makePlatform8Dof(this->legs);
    subscriberPlatform = node.subscribe("Catix/Platform8Dof", 1, &CatixKinematics::listenerPlatformState, this);
    ROS_INFO("8DOF platform listener is ready...");

    QObject::connect(&this->window, &SimulationWindow::onServoAngle, [this](size_t servoIndex, double servoAngle) 
    {
        catix_messages::ServoState servoStateMessage;
        servoStateMessage.servo_index = static_cast<uint8_t>(servoIndex);
        servoStateMessage.rotate_angle = servoAngle;

        this->publisherServo.publish(servoStateMessage);
        ROS_INFO("Servo %d: [%frad]", servoIndex, servoAngle);
    });

    window.show();
}

void CatixKinematics::listenerLegState(const catix_messages::TwoDofLegStateConstPtr &legState)
{
    const size_t legIndex = static_cast<size_t>(legState->leg_index);
    const geometry::PolarCoordinates targetPosition
    {
        legState->radial_coordinate, 
        legState->angular_coordinate 
    };

    if (legIndex >= this->legs.size())
    {
        ROS_ERROR("Leg %d: Can't set position as leg is not available", legIndex);
        return;
    }

    if (!this->legs[legIndex]->setPosition(targetPosition))
    {
        ROS_ERROR("Leg %d: Setting position failed", legIndex);
        return;
    }

    ROS_INFO("Leg %d: [%fm; %frad]",  legIndex, 
        targetPosition.radialCoordinate, targetPosition.angularCoordinate);
}

void CatixKinematics::listenerPlatformState(const catix_messages::EightDofPlatformStateConstPtr &platformState)
{
    const double moveForwardSpeed = platformState->move_forward_speed;
    const double rotateClockwiseSpeed = platformState->rotate_clockwise_speed;

    platform->setSpeed(moveForwardSpeed, rotateClockwiseSpeed);
    ROS_INFO("Platform: [%fm/s; %frad/s]",  moveForwardSpeed, rotateClockwiseSpeed);
}
