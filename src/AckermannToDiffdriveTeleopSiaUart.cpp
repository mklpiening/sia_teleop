#include "sia_teleop/AckermannToDiffdriveTeleopSiaUart.hpp"

#include <iostream>

AckermannToDiffdriveTeleopSiaUart::AckermannToDiffdriveTeleopSiaUart(std::string port,
                                                                     int baudRate,
                                                                     float maxSteeringWheelAngle,
                                                                     float maxThrottleState,
                                                                     float maxBrakeState,
                                                                     float linearAcceleration,
                                                                     float angularAcceleration,
                                                                     float maxLinearVelocity,
                                                                     float maxAngularVelocity)
    : AckermannToDiffdriveTeleop(
        linearAcceleration, maxAngularVelocity, maxLinearVelocity, maxAngularVelocity),
      m_sialib(port, baudRate), m_driveState(sialib::NEUTRAL)
{
    m_sialib.m_steeringWheelHandler = [this, maxSteeringWheelAngle](int angle) {
        setSteeringWheelAngle(-1 * static_cast<float>(angle) / maxSteeringWheelAngle);
    };

    m_sialib.m_throttleHandler = [this, maxThrottleState](int state) {
        m_throttle = static_cast<float>(state) / maxThrottleState;
        switch (m_driveState)
        {
            case sialib::DRIVE:
                setThrottle(m_throttle);
                break;

            case sialib::REVERSE:
                setThrottle(-m_throttle);
                break;

            default:
                setThrottle(0);
        }
    };

    m_sialib.m_driveStateHandler = [this](sialib::driveState_t state) {
        m_driveState = state;
    };

    /*m_sialib.m_brakeHandler = [this, maxBrakeState](int state) {
        m_brake = static_cast<float>(state) / maxBrakeState;
        setThrottle(m_throttle - m_brake);
    };*/

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sia_diffdrive_uart");

    AckermannToDiffdriveTeleopSiaUart teleop(
        "/dev/ttyACM0", 115200, 90.0, 100.0, 100.0, 0.8, 10, 1, 1);

    ros::spin();
}