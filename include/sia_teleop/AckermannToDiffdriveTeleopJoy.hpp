#ifndef ACKERMANN_TO_DIFFDRIVE_TELEOP_JOY_HPP
#define ACKERMANN_TO_DIFFDRIVE_TELEOP_JOY_HPP

#include "sia_teleop/AckermannToDiffdriveTeleop.hpp"

#include <sensor_msgs/Joy.h>

#define JOY_AXIS_LEFT_STICK_HORIZONTAL 0
#define JOY_AXIS_LEFT_STICK_VERTICAL 1
#define JOY_AXIS_LEFT_TRIGGER 5
#define JOY_AXIS_RIGHT_TRIGGER 4

/**
 * @brief allows a GTA style control of a diffdrive vehicle
 *
 * Subscribes to the joy topic and uses lthe controllers left and right trigger to control the
 * throttle of the vehicle. The left stick of the controller is used to steer the vehicle.
 */
class AckermannToDiffdriveTeleopJoy : public AckermannToDiffdriveTeleop
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
    AckermannToDiffdriveTeleopJoy(float linearAcceleration  = 1.0,
                                  float angularAcceleration = 1.0,
                                  float maxLinearVelocity   = 1.0,
                                  float maxAngularVelocity  = 1.0);

    /**
     * @brief Callback of the subscription to the joy topic
     *
     * This funtion gets called ecery time a joy input is received
     *
     * @param joy incoming joy message
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  private:
    ros::Subscriber m_joySubscriber;
};

#endif