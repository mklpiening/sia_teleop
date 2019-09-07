#include "sia_teleop/DiffdriveTeleop.hpp"

DiffdriveTeleop::DiffdriveTeleop(float linearAcceleration,
                                 float angularAcceleration,
                                 float maxLinearVelocity,
                                 float maxAngularVelocity)
    : m_linearAcceleration(linearAcceleration), m_angularAcceleration(angularAcceleration),
      m_maxLinearVelocity(maxLinearVelocity), m_maxAngularVelocity(maxAngularVelocity),
      m_targetAngularVelocity(0), m_targetLinearVelocity(0)
{
    m_velCommand.linear.x = 0;
    m_velCommand.linear.y = 0;
    m_velCommand.linear.z = 0;

    m_velCommand.angular.x = 0;
    m_velCommand.angular.y = 0;
    m_velCommand.angular.z = 0;

    m_velPublisher = m_nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    m_updateTimer
        = m_nodeHandle.createTimer(ros::Duration(0.1), &DiffdriveTeleop::updateVelocity, this);
}

void DiffdriveTeleop::setTargetVelocity(float targetLinearVelocity, float targetAngularVelocity)
{
    m_targetLinearVelocity  = targetLinearVelocity;
    m_targetAngularVelocity = targetAngularVelocity;
}

void DiffdriveTeleop::updateVelocity(const ros::TimerEvent& t_event)
{
    if (m_velCommand.linear.x < m_targetLinearVelocity)
    {
        m_velCommand.linear.x += m_linearAcceleration;

        if (m_velCommand.linear.x > m_maxLinearVelocity)
        {
            m_velCommand.linear.x = m_maxLinearVelocity;
        }

        if (m_velCommand.linear.x > m_targetLinearVelocity)
        {
            m_velCommand.linear.x = m_targetLinearVelocity;
        }
    }
    else if (m_velCommand.linear.x > m_targetLinearVelocity)
    {
        m_velCommand.linear.x -= m_linearAcceleration;

        if (m_velCommand.linear.x < -m_maxLinearVelocity)
        {
            m_velCommand.linear.x = -m_maxLinearVelocity;
        }

        if (m_velCommand.linear.x < m_targetLinearVelocity)
        {
            m_velCommand.linear.x = m_targetLinearVelocity;
        }
    }

    if (m_velCommand.angular.z < m_targetAngularVelocity)
    {
        m_velCommand.angular.z += m_angularAcceleration;

        if (m_velCommand.angular.z > m_maxAngularVelocity)
        {
            m_velCommand.angular.z = m_maxAngularVelocity;
        }

        if (m_velCommand.angular.z > m_targetAngularVelocity)
        {
            m_velCommand.angular.z = m_targetAngularVelocity;
        }
    }
    else if (m_velCommand.angular.z > m_targetAngularVelocity)
    {
        m_velCommand.angular.z -= m_angularAcceleration;

        if (m_velCommand.angular.z < -m_maxAngularVelocity)
        {
            m_velCommand.angular.z = -m_maxAngularVelocity;
        }

        if (m_velCommand.angular.z < m_targetAngularVelocity)
        {
            m_velCommand.angular.z = m_targetAngularVelocity;
        }
    }

    m_velPublisher.publish(m_velCommand);
}