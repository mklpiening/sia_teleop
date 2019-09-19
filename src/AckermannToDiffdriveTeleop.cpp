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
      m_msgReceived(false), m_speed(0), m_steeringAngle(0), m_throttle(0), m_brake(0)
{
    m_updateMovementTimer = m_nodeHandle.createTimer(
        ros::Duration(0.01), &AckermannToDiffdriveTeleop::updateDiffdriveVelocity, this);
}

void AckermannToDiffdriveTeleop::setSteeringWheelAngle(float angle)
{
    m_steeringAngle = angle;
    m_steeringAngle = std::max(m_steeringAngle, -1.0f);
    m_steeringAngle = std::min(m_steeringAngle, 1.0f);

    m_msgReceived = true;
}

void AckermannToDiffdriveTeleop::setThrottle(float throttle)
{
    m_throttle = throttle;
    m_throttle = std::max(m_throttle, -1.0f);
    m_throttle = std::min(m_throttle, 1.0f);

    m_msgReceived = true;
}

void AckermannToDiffdriveTeleop::setBrake(float brake)
{
    m_brake = brake;
    m_brake = std::max(m_brake, 0.0f);
    m_brake = std::min(m_brake, 1.0f);

    m_msgReceived = true;
}

void AckermannToDiffdriveTeleop::updateDiffdriveVelocity(const ros::TimerEvent& t_event)
{
    if (m_msgReceived)
    {
        float speedBefore = m_speed;

        if (m_speed > 0)
        {
            // slow the vehicle down in small steps
            m_speed -= m_defaultDeceleration / 100;

            // handle breaking
            m_speed -= m_maxBrakeDeceleration / 100 * m_brake;
        }
        else if (m_speed < 0)
        {
            // slow the vehicle down in small steps
            m_speed += m_defaultDeceleration / 100;

            // handle breaking
            m_speed += m_maxBrakeDeceleration / 100 * m_brake;
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

        float targetSpeed  = m_throttle;
        float acceleration = std::abs(m_throttle) * m_maxAcceleration / 100;

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

        float targetLinearVelocity
            = m_speed * m_maxLinearVelocity * (1 - 0.5 * abs(m_steeringAngle));
        float targetAngularVelocity = m_speed * m_steeringAngle * m_maxAngularVelocity;

        setVelocity(targetLinearVelocity, targetAngularVelocity);
    }
    m_msgReceived = false;
}