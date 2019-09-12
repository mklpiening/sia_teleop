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
     * @param linearAcceleration acceleration for linear velocity in (intensity / s)
     * @param angularAcceleration acceleration for rotation in (intensity / s)
     * @param maxLinearVelocity maximum allowed velocity for linear movelemt
     * @param maxAngularVelocity maximum allowed velocity for rotations
     */
    DiffdriveTeleop(float linearAcceleration  = 1.0,
                    float angularAcceleration = 1.0,
                    float maxLinearVelocity   = 1.0,
                    float maxAngularVelocity  = 1.0);

    /**
     * @brief sets the target angular and linear velocity
     *
     * Sets the angular and linear velocity for the vehicle. The vehicle will start to accelerate at the
     * giben acceleration until the target velocity is reached.
     *
     * @param targetLinearVelocity targeted linear velocity
     * @param targetAngularVelocity targeted angular velocity
     */
    void setTargetVelocity(float targetLinearVelocity, float targetAngularVelocity);

    /**
     * @brief callback of timer that updates the velocity according to the given accalerations and
     * publishes the new velocities to protect motors and gearboxes
     *
     * @param t_event timer event
     */
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