#include "sia_teleop/AckermannToDiffdriveTeleopJoy.hpp"

#include <iostream>

AckermannToDiffdriveTeleopJoy::AckermannToDiffdriveTeleopJoy(float maxAcceleration,
                                                             float defaultDeceleration,
                                                             float maxBrakeDeceleration,
                                                             float maxLinearVelocity,
                                                             float maxAngularVelocity)
    : AckermannToDiffdriveTeleop(maxAcceleration,
                                 defaultDeceleration,
                                 maxBrakeDeceleration,
                                 maxLinearVelocity,
                                 maxAngularVelocity)
{
    m_joySubscriber = m_nodeHandle.subscribe<sensor_msgs::Joy>(
        "joy", 15, &AckermannToDiffdriveTeleopJoy::joyCallback, this);
}

void AckermannToDiffdriveTeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    float leftStickHorizontal = joy->axes[JOY_AXIS_LEFT_STICK_HORIZONTAL];
    float rightTrigger        = (1 - joy->axes[JOY_AXIS_RIGHT_TRIGGER]) / 2;
    float leftTrigger         = (1 - joy->axes[JOY_AXIS_LEFT_TRIGGER]) / 2;

    setSteeringWheelAngle(leftStickHorizontal);
    setThrottle(rightTrigger - leftTrigger);
    //setBrake(leftTrigger);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sia_diffdrive_joy");

    AckermannToDiffdriveTeleopJoy teleop(1, 0.1, 1, 1, 1);

    ros::spin();
}