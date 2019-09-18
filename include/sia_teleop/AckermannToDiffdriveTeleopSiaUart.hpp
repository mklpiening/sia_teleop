#ifndef ACKERMANN_TO_DIFFDRIVE_TELEOP_SIA_UART
#define ACKERMANN_TO_DIFFDRIVE_TELEOP_SIA_UART

#include "sia_teleop/AckermannToDiffdriveTeleopSia.hpp"

class AckermannToDiffdriveTeleopSiaUart : public AckermannToDiffdriveTeleopSia
{
  public:
    /**
     * @brief construct a new diffdrive teleop object
     *
     * @param port mountpoint of the target device
     * @param baudRate baud rate for the uart communication with the target device
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
                                      float maxAngularVelocity    = 1.0,
                                      int numGears                = 1);
};

#endif // ACKERMANN_TO_DIFFDRIVE_TELEOP_SIA_UART