#pragma once

#include "servo/ServoInterface.h"
#include "limb/Limb2DofInterface.h"
#include "platform/PlatformInterface.h"

#include <ros/ros.h>

#include <catix_messages/EightDofPlatformState.h>
#include <catix_messages/TwoDofLegState.h>

//------------------------------------------------------------------------

class CatixKinematics 
{
    public:
        CatixKinematics();

    private:
        void listenerPlatformState(const catix_messages::EightDofPlatformStateConstPtr &rPlatformState);
        void listenerLegState(const catix_messages::TwoDofLegStateConstPtr &rLegState);

    private:
        ros::NodeHandle node;
        ros::Subscriber subscriberPlatform;
        ros::Subscriber subscriberLeg;
        ros::Publisher publisherServo;

    private:
        std::vector<std::shared_ptr<servo::IServo>> servos;
        std::vector<std::shared_ptr<limb::ILimb2Dof>> legs;
        std::unique_ptr<platform::IPlatform> platform;
};
