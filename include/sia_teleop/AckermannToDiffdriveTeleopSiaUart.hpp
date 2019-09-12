#ifndef ACKERMANN_TO_DIFFDRIVE_TELEOP_SIA_HPP
#define ACKERMANN_TO_DIFFDRIVE_TELEOP_SIA_HPP

#include "sia_teleop/AckermannToDiffdriveTeleop.hpp"

#include <sialib/SialibUart.hpp>

class AckermannToDiffdriveTeleopSiaUart : public AckermannToDiffdriveTeleop
{
  public:
    /**
     * @brief construct a new diffdrive teleop object
     *
     * @param port serial port for communication
     * @param baudRate baud rate for serial port
     * @param maxSteeringWheelAngle steering wheel angle for maximum steering angle
     * @param maxThrottleState value for full throttle
     * @param maxBreakState value for full brake
     * @param linearAcceleration acceleration for linear velocity in (intensity / s)
     * @param angularAcceleration acceleration for rotation in (intensity / s)
     * @param maxLinearVelocity maximum allowed velocity for linear movelemt
     * @param maxAngularVelocity maximum allowed velocity for rotations
     */
    AckermannToDiffdriveTeleopSiaUart(std::string port            = "/dev/ttyACM0",
                                      int baudRate                = 115200,
                                      float maxSteeringWheelAngle = 90.0,
                                      float maxThrottleState      = 100.0,
                                      float maxBrakeState         = 100.0,
                                      float linearAcceleration    = 1.0,
                                      float angularAcceleration   = 1.0,
                                      float maxLinearVelocity     = 1.0,
                                      float maxAngularVelocity    = 1.0);

  private:
    sialib::SialibUart m_sialib;

    float m_throttle;
    sialib::driveState_t m_driveState;
};

#endif