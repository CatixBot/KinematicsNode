#include "CatixKinematics.h"

#include "platform/PlatformParameters.h"
#include "platform/Platform8Dof.h"
#include "limb/LimbSegment.h"
#include "limb/PolarCoordinates.h"
#include "limb/Leg2Dof.h"
#include "servo/ServoPublisher.h"

#include <cmath>
#include <std_msgs/Empty.h>
#include <catix_messages/ServoState.h>

//---------------------------------------------------------------------------

#define NUMBER_OF_JOINTS 8

//---------------------------------------------------------------------------

std::vector<std::shared_ptr<servo::IServo>> makeJoints(size_t numberOfJoints, ros::NodeHandle &node)
{
    std::vector<std::shared_ptr<servo::IServo>> joints;

    for (size_t jointIndex = 0; jointIndex < numberOfJoints; ++jointIndex)
    {
        joints.emplace_back(std::make_shared<servo::ServoPublisher>(jointIndex, node));
    }

    return joints;
}

std::vector<std::shared_ptr<limb::ILimb2Dof>> makeLegs2Dof(const std::vector<std::shared_ptr<servo::IServo>>& joints)
{
    const size_t EXPECTED_NUMBER_OF_JOINTS = 8;
    const size_t PROVIDED_NUMBER_OF_JOINTS = joints.size();

    if (PROVIDED_NUMBER_OF_JOINTS != EXPECTED_NUMBER_OF_JOINTS)
    {
        ROS_ERROR("Legs can't be constructed as %d joints provided, but %d joints expected", 
            PROVIDED_NUMBER_OF_JOINTS, EXPECTED_NUMBER_OF_JOINTS);
        return {};
    }

    std::vector<std::shared_ptr<limb::ILimb2Dof>> legs;

    const double LOWER_SEGMENT_LENGTH_METERS = 0.060;
    const double UPPER_SEGMENT_LENGTH_METERS = 0.049;

    size_t legIndex = 0;
    limb::LimbSegment frontLeftLowerSegment;
    frontLeftLowerSegment.jointServo = joints[0];
    frontLeftLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment frontLeftUpperSegment;
    frontLeftUpperSegment.jointServo = joints[1];
    frontLeftUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto frontLeftLeg = std::make_shared<limb::Leg2Dof>(legIndex, frontLeftLowerSegment, frontLeftUpperSegment);
    legs.push_back(frontLeftLeg);
    
    legIndex = 1;
    limb::LimbSegment frontRightLowerSegment;
    frontRightLowerSegment.jointServo = joints[2];
    frontRightLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment frontRightUpperSegment;
    frontRightUpperSegment.jointServo = joints[3];
    frontRightUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto frontRightLeg = std::make_shared<limb::Leg2Dof>(legIndex, frontRightLowerSegment, frontRightUpperSegment);
    legs.push_back(frontRightLeg);

    legIndex = 2;
    limb::LimbSegment rearRightLowerSegment;
    rearRightLowerSegment.jointServo = joints[4];
    rearRightLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment rearRightUpperSegment;
    rearRightUpperSegment.jointServo = joints[5];
    rearRightUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto rearRightLeg = std::make_shared<limb::Leg2Dof>(legIndex, rearRightLowerSegment, rearRightUpperSegment);
    legs.push_back(rearRightLeg);

    legIndex = 3;
    limb::LimbSegment rearLeftLowerSegment;
    rearLeftLowerSegment.jointServo = joints[6];
    rearLeftLowerSegment.linkLength = LOWER_SEGMENT_LENGTH_METERS;
    limb::LimbSegment rearLeftUpperSegment;
    rearLeftUpperSegment.jointServo = joints[7];
    rearLeftUpperSegment.linkLength = UPPER_SEGMENT_LENGTH_METERS;
    auto rearLeftLeg = std::make_shared<limb::Leg2Dof>(legIndex, rearLeftLowerSegment, rearLeftUpperSegment);
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
    ROS_INFO("Create drop all joints publisher");
    this->publisherAllJointsDrop = node.advertise<std_msgs::Empty>("Catix/SignalingDrop", 1);

    ROS_INFO("Create joint publishers");
    this->joints = makeJoints(NUMBER_OF_JOINTS, this->node);

    ROS_INFO("Create 2DOF leg subscriber");
    this->legs = makeLegs2Dof(this->joints);
    subscriberLeg = node.subscribe("Catix/Leg2Dof", 1, &CatixKinematics::listenerLegState, this);

    ROS_INFO("Create 8DOF platform subscriber");
    this->platform = makePlatform8Dof(this->legs);
    subscriberPlatform = node.subscribe("Catix/Platform8Dof", 1, &CatixKinematics::listenerPlatformState, this);

    QObject::connect(&this->window, &SimulationWindow::onDropAllJoints, [this]()
    {
        std_msgs::Empty dropAllJointsEventMessage;
        this->publisherAllJointsDrop.publish(dropAllJointsEventMessage);
        ROS_INFO("Drop all joints");
    });

    QObject::connect(&this->window, &SimulationWindow::onJointAngle, [this](size_t servoIndex, double servoAngle) 
    {
        this->joints[servoIndex]->setAngle(servoAngle);
    });

    QObject::connect(&this->window, &SimulationWindow::onLegPosition, [this](size_t legIndex, double radialCoordinate, double angularCoordinate)
    {
        geometry::PolarCoordinates coordinates;
        coordinates.radialCoordinate = radialCoordinate;
        coordinates.angularCoordinate = angularCoordinate;
        this->legs[legIndex]->setPosition(coordinates);
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
