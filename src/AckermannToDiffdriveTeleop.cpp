#include "sia_teleop/AckermannToDiffdriveTeleop.hpp"

AckermannToDiffdriveTeleop::AckermannToDiffdriveTeleop(float linearAcceleration,
                                                       float angularAcceleration,
                                                       float maxLinearVelocity,
                                                       float maxAngularVelocity)
    : DiffdriveTeleop(
        linearAcceleration, angularAcceleration, maxLinearVelocity, maxAngularVelocity),
      m_steeringAngle(0), m_throttle(0)
{
}

void AckermannToDiffdriveTeleop::setSteeringWheelAngle(float angle)
{
    m_steeringAngle = angle;

    applyDiffdriveVelocity();
}

void AckermannToDiffdriveTeleop::setThrottle(float throttle)
{
    m_throttle = throttle;

    applyDiffdriveVelocity();
}

void AckermannToDiffdriveTeleop::applyDiffdriveVelocity()
{
    setTargetVelocity(m_throttle * m_maxLinearVelocity * (1 - 0.5 * abs(m_steeringAngle)), 
                      m_throttle * m_steeringAngle * m_maxAngularVelocity);
}