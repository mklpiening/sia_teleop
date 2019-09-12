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
     * @param maxBrakeState value for full brake
     * @param maxAcceleration acceleration at its peak in (intensity / s)
     * @param defaultDeceleration deceleration when rolling
     * @param maxBrakeDeceleration deceleration whith maximum brake state
     * @param maxLinearVelocity maximum linear velocity
     * @param maxAngularVelocity maximum angular velocity
     */
    AckermannToDiffdriveTeleopSiaUart(std::string port            = "/dev/ttyACM0",
                                      int baudRate                = 115200,
                                      float maxSteeringWheelAngle = 90.0,
                                      float maxThrottleState      = 100.0,
                                      float maxBrakeState         = 100.0,
                                      float maxAcceleration       = 1.0,
                                      float defaultDeceleration   = 0.1,
                                      float maxBrakeDeceleration  = 1.0,
                                      float maxLinearVelocity     = 1.0,
                                      float maxAngularVelocity    = 1.0);

  private:
    sialib::SialibUart m_sialib;

    float m_steeringWheelAngle;
    float m_throttle;
    float m_brake;
    sialib::driveState_t m_driveState;
};

#endif