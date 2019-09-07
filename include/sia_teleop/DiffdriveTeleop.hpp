#ifndef DIFFDRIVE_TELEOP_HPP
#define DIFFDRIVE_TELEOP_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class DiffdriveTeleop
{
  public:
    DiffdriveTeleop(float linearAcceleration  = 0.1,
                    float angularAcceleration = 0.1,
                    float maxLinearVelocity   = 1.0,
                    float maxAngularVelocity  = 1.0);

    void setTargetVelocity(float targetLinearVelocity, float targetAngularVelocity);

    void updateVelocity(const ros::TimerEvent& t_event);

  protected:
    float m_maxLinearVelocity;
    float m_maxAngularVelocity;

    ros::NodeHandle m_nodeHandle;

  private:
    ros::Publisher m_velPublisher;
    geometry_msgs::Twist m_velCommand;

    ros::Timer m_updateTimer;

    float m_linearAcceleration;
    float m_angularAcceleration;

    float m_targetLinearVelocity;
    float m_targetAngularVelocity;
};

#endif