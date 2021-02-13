#include "limb/Leg2Dof.h"

#include <cmath>
#include <ros/ros.h>

//------------------------------------------------------------------------

const double DENOMINATOR_THRESHOLD = 0.001;

//------------------------------------------------------------------------

namespace geometry
{
    inline double calcSquare(double value)
    {
        return value*value;
    }

    double calcLowerJointAngle(double firstLinkLength, double secondLinkLength,
        PolarCoordinates targetCoordinates)
    {
        if (targetCoordinates.radialCoordinate < DENOMINATOR_THRESHOLD)
        {
            targetCoordinates.radialCoordinate = DENOMINATOR_THRESHOLD;
        }

        /*
         * phi_2 = arccos((l_1^2 + l_2^2 - p_t^2)/(2*l_1*l_2))
         */
        return std::acos((calcSquare(firstLinkLength) + calcSquare(secondLinkLength) - calcSquare(targetCoordinates.radialCoordinate)) /
            (2 * firstLinkLength * secondLinkLength));
    }

    double calcUpperJointAngle(double firstLinkLength, double secondLinkLength,
        PolarCoordinates targetCoordinates)
    {
        /*
         * phi_1 = arccos((l_1^2 + p_t^2 - l_2^2)/(2*l_1*p_t)) + phi
         */
        return std::acos((calcSquare(firstLinkLength) + calcSquare(targetCoordinates.radialCoordinate) - calcSquare(secondLinkLength)) /
            (2 * firstLinkLength * targetCoordinates.radialCoordinate)) + targetCoordinates.angularCoordinate;
    }
}

//------------------------------------------------------------------------

limb::Leg2Dof::Leg2Dof(size_t legIndex, limb::LimbSegment lowerSegment, limb::LimbSegment upperSegment)
    : legIndex(legIndex)
    , lowerSegment(lowerSegment)
    , upperSegment(upperSegment)
{
}

bool limb::Leg2Dof::setPosition(geometry::PolarCoordinates targetCoordinates)
{
    const double phi2 = calcLowerJointAngle(this->lowerSegment.linkLength, 
        this->upperSegment.linkLength, targetCoordinates);
    if (!this->lowerSegment.jointServo->setAngle(phi2))
    {
        return false;
    }

    const double phi1 = calcUpperJointAngle(this->lowerSegment.linkLength, 
        this->upperSegment.linkLength, targetCoordinates);
    if (!this->upperSegment.jointServo->setAngle(phi1))
    {
        return false;
    }

    ROS_INFO("Leg %d: [%frad; %frad]", this->legIndex, phi1, phi2);
    return true;
}
