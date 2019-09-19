#include "sia_teleop/AckermannToDiffdriveTeleopSia.hpp"

#include <iostream>

AckermannToDiffdriveTeleopSia::AckermannToDiffdriveTeleopSia(std::shared_ptr<sialib::Sialib> sialib,
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
      m_sialib(sialib), m_numGears(numGears), m_currentGear(0),
      m_maxMaxLinearVelocity(maxLinearVelocity), m_maxMaxAngularVelocity(maxAngularVelocity),
      m_steeringWheelAngle(0), m_throttle(0), m_brake(0), m_driveState(sialib::NEUTRAL),
      m_leftLeverState(false), m_rightLeverState(false)
{
    m_sialib->steeringWheelHandler = [this, maxSteeringWheelAngle](int angle) {
        m_steeringWheelAngle = -1 * static_cast<float>(angle) / maxSteeringWheelAngle;
        setSteeringWheelAngle(m_steeringWheelAngle);
    };

    m_sialib->throttleHandler = [this, maxThrottleState](int state) {
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

    m_sialib->driveStateHandler = [this](sialib::DriveState state) {
        if (state != sialib::SHIFT_UP && state != sialib::SHIFT_DOWN && state != sialib::MANUAL)
        {
            m_driveState = state;
        }
    };

    m_sialib->brakeHandler = [this, maxBrakeState](int state) {
        m_brake = static_cast<float>(state) / maxBrakeState;
        setBrake(m_brake);
    };

    m_sialib->shiftUpHandler = [this]() {
        m_currentGear++;
        updateGear();
    };

    m_sialib->shiftDownHandler = [this]() {
        m_currentGear--;
        updateGear();
    };

    m_sialib->rightLeverHandler = [this](bool pulled) {
        if (pulled && !m_rightLeverState)
        {
            m_currentGear++;
            updateGear();
        }
        m_rightLeverState = pulled;
    };

    m_sialib->leftLeverHandler = [this](bool pulled) {
        if (pulled && !m_leftLeverState)
        {
            m_currentGear--;
            updateGear();
        }
        m_leftLeverState = pulled;
    };
}

void AckermannToDiffdriveTeleopSia::updateGear()
{
    m_currentGear = std::max(0, m_currentGear);
    m_currentGear = std::min(m_numGears - 1, m_currentGear);

    std::cout << "Gear: " << m_currentGear << std::endl;

    m_maxAngularVelocity = (m_maxMaxAngularVelocity * (m_currentGear + 1)) / m_numGears;
    m_maxLinearVelocity  = (m_maxMaxLinearVelocity * (m_currentGear + 1)) / m_numGears;
}
