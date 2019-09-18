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
    // setBrake(leftTrigger);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sia_diffdrive_joy");

    ros::NodeHandle nhPrivate("~");

    float maxAcceleration;
    nhPrivate.param("max_acceleration", maxAcceleration, 1.5f);
    float defaultDeceleration;
    nhPrivate.param("default_deceleration", defaultDeceleration, 0.4f);
    float maxBrakeDeceleration;
    nhPrivate.param("max_brake_deceleration", maxBrakeDeceleration, 2.0f);
    float maxLinearVelocity;
    nhPrivate.param("max_linear_velocity", maxLinearVelocity, 1.0f);
    float maxAngularVelocity;
    nhPrivate.param("max_angular_velocity", maxAngularVelocity, 1.0f);

    AckermannToDiffdriveTeleopJoy teleop(maxAcceleration,
                                         defaultDeceleration,
                                         maxBrakeDeceleration,
                                         maxLinearVelocity,
                                         maxAngularVelocity);

    ros::spin();
}