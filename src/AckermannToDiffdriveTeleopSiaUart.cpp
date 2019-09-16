#include "sia_teleop/AckermannToDiffdriveTeleopSiaUart.hpp"

#include <iostream>

AckermannToDiffdriveTeleopSiaUart::AckermannToDiffdriveTeleopSiaUart(std::string port,
                                                                     int baudRate,
                                                                     float maxSteeringWheelAngle,
                                                                     float maxThrottleState,
                                                                     float maxBrakeState,
                                                                     float maxAcceleration,
                                                                     float defaultDeceleration,
                                                                     float maxBrakeDeceleration,
                                                                     float maxLinearVelocity,
                                                                     float maxAngularVelocity,
                                                                     int numGears)
    : AckermannToDiffdriveTeleop(maxAcceleration,
                                 defaultDeceleration,
                                 maxBrakeDeceleration,
                                 maxLinearVelocity / numGears,
                                 maxAngularVelocity / numGears),
      m_sialib(port, baudRate), m_numGears(numGears), m_currentGear(0),
      m_maxMaxLinearVelocity(maxLinearVelocity), m_maxMaxAngularVelocity(maxAngularVelocity),
      m_steeringWheelAngle(0), m_throttle(0), m_brake(0), m_driveState(sialib::NEUTRAL)
{
    m_sialib.steeringWheelHandler = [this, maxSteeringWheelAngle](int angle) {
        m_steeringWheelAngle = -1 * static_cast<float>(angle) / maxSteeringWheelAngle;
        setSteeringWheelAngle(m_steeringWheelAngle);
    };

    m_sialib.throttleHandler = [this, maxThrottleState](int state) {
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

    m_sialib.driveStateHandler = [this](sialib::DriveState state) {
        if (state != sialib::SHIFT_UP && state != sialib::SHIFT_DOWN && state != sialib::MANUAL)
        {
            m_driveState = state;
        }
    };

    m_sialib.brakeHandler = [this, maxBrakeState](int state) {
        m_brake = static_cast<float>(state) / maxBrakeState;
        setBrake(m_brake);
    };

    m_sialib.shiftUpHandler = [this]() {
        m_currentGear++;
        m_currentGear = std::min(m_numGears - 1, m_currentGear);
        std::cout << "Gear: " << m_currentGear << std::endl;
        m_maxAngularVelocity = (m_maxMaxAngularVelocity * (m_currentGear + 1)) / m_numGears;
        m_maxLinearVelocity  = (m_maxMaxLinearVelocity * (m_currentGear + 1)) / m_numGears;
    };

    m_sialib.shiftDownHandler = [this]() {
        m_currentGear--;
        m_currentGear = std::max(0, m_currentGear);
        std::cout << "Gear: " << m_currentGear << std::endl;
        m_maxAngularVelocity = (m_maxMaxAngularVelocity * (m_currentGear + 1)) / m_numGears;
        m_maxLinearVelocity  = (m_maxMaxLinearVelocity * (m_currentGear + 1)) / m_numGears;
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sia_diffdrive_uart");

    AckermannToDiffdriveTeleopSiaUart teleop(
        "/dev/ttyACM0", 115200, 90.0, 100.0, 100.0, 1.5, 0.4, 2, 1, 1, 3);

    ros::spin();
}