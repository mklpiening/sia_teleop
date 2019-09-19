#ifndef ACKERMANN_TO_DIFFDRIVE_TELEOP_SIA_HPP
#define ACKERMANN_TO_DIFFDRIVE_TELEOP_SIA_HPP

#include "sia_teleop/AckermannToDiffdriveTeleop.hpp"

#include <memory>
#include <sialib/Sialib.hpp>

class AckermannToDiffdriveTeleopSia : public AckermannToDiffdriveTeleop
{
  public:
    /**
     * @brief construct a new diffdrive teleop object
     *
     * @param sialib sialib instance
     * @param maxSteeringWheelAngle steering wheel angle for maximum steering angle
     * @param maxThrottleState value for full throttle
     * @param maxBrakeState value for full brake
     * @param maxAcceleration acceleration at its peak in (intensity / s)
     * @param defaultDeceleration deceleration when rolling
     * @param maxBrakeDeceleration deceleration whith maximum brake state
     * @param maxLinearVelocity maximum linear velocity
     * @param maxAngularVelocity maximum angular velocity
     */
    AckermannToDiffdriveTeleopSia(std::shared_ptr<sialib::Sialib> sialib,
                                  float maxSteeringWheelAngle = 90.0,
                                  float maxThrottleState      = 100.0,
                                  float maxBrakeState         = 100.0,
                                  float maxAcceleration       = 1.0,
                                  float defaultDeceleration   = 0.1,
                                  float maxBrakeDeceleration  = 1.0,
                                  float maxLinearVelocity     = 1.0,
                                  float maxAngularVelocity    = 1.0,
                                  int numGears                = 1);

  private:
    void updateGear();

    std::shared_ptr<sialib::Sialib> m_sialib;

    int m_numGears;
    int m_currentGear;
    float m_maxMaxLinearVelocity;
    float m_maxMaxAngularVelocity;

    float m_steeringWheelAngle;
    float m_throttle;
    float m_brake;
    sialib::DriveState m_driveState;
    bool m_leftLeverState;
    bool m_rightLeverState;
};

#endif