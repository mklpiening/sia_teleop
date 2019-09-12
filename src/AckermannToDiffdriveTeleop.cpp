#include "sia_teleop/AckermannToDiffdriveTeleop.hpp"

#include <algorithm>
#include <iostream>

AckermannToDiffdriveTeleop::AckermannToDiffdriveTeleop(float maxAcceleration,
                                                       float defaultDeceleration,
                                                       float maxBrakeDeceleration,
                                                       float maxLinearVelocity,
                                                       float maxAngularVelocity)
    : DiffdriveTeleop(maxLinearVelocity, maxAngularVelocity), m_maxAcceleration(maxAcceleration),
      m_defaultDeceleration(defaultDeceleration), m_maxBrakeDeceleration(maxBrakeDeceleration),
      m_speed(0), m_steeringAngle(0), m_throttle(0), m_brake(0)
{
    m_updateMovementTimer = m_nodeHandle.createTimer(
        ros::Duration(0.1), &AckermannToDiffdriveTeleop::updateDiffdriveVelocity, this);
}

void AckermannToDiffdriveTeleop::setSteeringWheelAngle(float angle)
{
    m_steeringAngle = angle;
    m_steeringAngle = std::max(m_steeringAngle, -1.0f);
    m_steeringAngle = std::min(m_steeringAngle, 1.0f);
}

void AckermannToDiffdriveTeleop::setThrottle(float throttle)
{
    m_throttle = throttle;
    m_throttle = std::max(m_throttle, -1.0f);
    m_throttle = std::min(m_throttle, 1.0f);
}

void AckermannToDiffdriveTeleop::setBrake(float brake)
{
    m_brake = brake;
    m_brake = std::max(m_brake, 0.0f);
    m_brake = std::min(m_brake, 1.0f);
}

void AckermannToDiffdriveTeleop::updateDiffdriveVelocity(const ros::TimerEvent& t_event)
{
    float speedBefore = m_speed;

    if (m_speed > 0)
    {
        // slow the vehicle down in small steps
        m_speed -= m_defaultDeceleration / 10;

        // handle breaking
        m_speed -= m_maxBrakeDeceleration / 10 * m_brake;
    }
    else if (m_speed < 0)
    {
        // slow the vehicle down in small steps
        m_speed += m_defaultDeceleration / 10;

        // handle breaking
        m_speed += m_maxBrakeDeceleration / 10 * m_brake;
    }

    // dont overshoot the deceleration goal
    if (speedBefore < 0)
    {
        m_speed = std::min(m_speed, 0.0f);
    }
    else
    {
        m_speed = std::max(m_speed, 0.0f);
    }

    float targetSpeed = m_throttle * m_maxLinearVelocity * (1 - 0.5 * abs(m_steeringAngle));
    float acceleration = std::abs(m_throttle) * m_maxAcceleration / 10;

    if ((m_speed < 0 && targetSpeed > 0) || (m_speed > 0 && targetSpeed < 0)
        || std::abs(m_speed) < std::abs(targetSpeed))
    {
        if (m_speed < targetSpeed)
        {
            m_speed += acceleration;

            m_speed = std::min(m_speed, targetSpeed);
        }
        else if (m_speed > targetSpeed)
        {
            m_speed -= acceleration;

            m_speed = std::max(m_speed, targetSpeed);
        }
    }

    float targetAngularVelocity
        = (m_speed / m_maxLinearVelocity) * m_steeringAngle * m_maxAngularVelocity;

    setVelocity(m_speed, targetAngularVelocity);
}