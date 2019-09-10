#include "sia_teleop/AckermannToDiffdriveTeleopSiA.hpp"

#include <iostream>

AckermannToDiffdriveTeleopSiA::AckermannToDiffdriveTeleopSiA(float linearAcceleration,
                                                             float angularAcceleration,
                                                             float maxLinearVelocity,
                                                             float maxAngularVelocity)
    : AckermannToDiffdriveTeleop(
        linearAcceleration, maxAngularVelocity, maxLinearVelocity, maxAngularVelocity),
      m_sialib("/dev/ttyACM0", 115200)
{
    // FOR TESTING: control the robot using only the steering wheel
    m_sialib.m_steeringWheelHandler = [this](int angle) {
        if (angle != 0)
        {
            setSteeringWheelAngle(static_cast<float>(angle + 360) / 90);
        }
    };

    m_sialib.m_leftLeverHandler = [this](bool state) { setThrottle(state ? -1 : 0); };

    m_sialib.m_rightLeverHandler = [this](bool state) { setThrottle(state ? 1 : 0); };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sia_diffdrive_sia");

    AckermannToDiffdriveTeleopSiA teleop(0.8, 10, 0.4, 0.6);

    ros::spin();
}