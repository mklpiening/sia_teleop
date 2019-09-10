#ifndef ACKERMANN_TO_DIFFDRIVE_TELEOP_SIA_HPP
#define ACKERMANN_TO_DIFFDRIVE_TELEOP_SIA_HPP

#include "sia_teleop/AckermannToDiffdriveTeleop.hpp"

#include <sialib/SialibUart.hpp>

class AckermannToDiffdriveTeleopSiA : public AckermannToDiffdriveTeleop
{
  public:
    /**
     * @brief construct a new diffdrive teleop object
     *
     * @param linearAcceleration acceleration for linear velocity in (intensity / s)
     * @param angularAcceleration acceleration for rotation in (intensity / s)
     * @param maxLinearVelocity maximum allowed velocity for linear movelemt
     * @param maxAngularVelocity maximum allowed velocity for rotations
     */
    AckermannToDiffdriveTeleopSiA(float linearAcceleration  = 1.0,
                                  float angularAcceleration = 1.0,
                                  float maxLinearVelocity   = 1.0,
                                  float maxAngularVelocity  = 1.0);
  private:
    sialib::SialibUart m_sialib;
};

#endif