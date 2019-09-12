#ifndef DIFFDRIVE_TELEOP_HPP
#define DIFFDRIVE_TELEOP_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/**
 * @brief publishes velocity commands for diffdrive vehicle
 */
class DiffdriveTeleop
{
  public:
    /**
     * @brief construct a new diffdrive teleop object
     *
     * @param maxLinearVelocity maximum linear velocity
     * @param maxAngularVelocity maximum angular velocity
     */
    DiffdriveTeleop(float maxLinearVelocity   = 1.0,
                    float maxAngularVelocity  = 1.0);

    /**
     * @brief sets the angular and velocity
     *
     * @param linearVelocity linear velocity
     * @param angularVelocity angular velocity
     */
    void setVelocity(float linearVelocity, float angularVelocity);

  protected:
    float m_maxLinearVelocity;
    float m_maxAngularVelocity;

    ros::NodeHandle m_nodeHandle;

  private:
    ros::Publisher m_velPublisher;
    geometry_msgs::Twist m_velCommand;
};

#endif