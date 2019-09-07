#ifndef ACKERMANN_TO_DIFFDRIVE_TELEOP_JOY_HPP
#define ACKERMANN_TO_DIFFDRIVE_TELEOP_JOY_HPP

#include "sia_teleop/AckermannToDiffdriveTeleop.hpp"

#include <sensor_msgs/Joy.h>

#define JOY_AXIS_LEFT_STICK_HORIZONTAL 0
#define JOY_AXIS_LEFT_STICK_VERTICAL 1
#define JOY_AXIS_LEFT_TRIGGER 5
#define JOY_AXIS_RIGHT_TRIGGER 4

class AckermannToDiffdriveTeleopJoy : public AckermannToDiffdriveTeleop
{
  public:
    AckermannToDiffdriveTeleopJoy(float linearAcceleration  = 0.1,
                                  float angularAcceleration = 0.1,
                                  float maxLinearVelocity   = 1.0,
                                  float maxAngularVelocity  = 1.0);

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  private:
    ros::Subscriber m_joySubscriber;
};

#endif