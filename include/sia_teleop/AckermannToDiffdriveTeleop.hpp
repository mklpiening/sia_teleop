#ifndef ACKERMANN_TO_DIFFDRIVE_TELEOP_HPP
#define ACKERMANN_TO_DIFFDRIVE_TELEOP_HPP

#include "sia_teleop/DiffdriveTeleop.hpp"

class AckermannToDiffdriveTeleop : public DiffdriveTeleop
{
  public:
    AckermannToDiffdriveTeleop(float linearAcceleration  = 0.1,
                               float angularAcceleration = 0.1,
                               float maxLinearVelocity   = 1.0,
                               float maxAngularVelocity  = 1.0);

    void setSteeringWheelAngle(float angle);

    void setThrottle(float throttle);

  private:
    void applyDiffdriveVelocity();

    float m_steeringAngle;
    float m_throttle;
};

#endif