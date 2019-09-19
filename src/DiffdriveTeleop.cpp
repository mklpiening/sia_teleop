#include "sia_teleop/DiffdriveTeleop.hpp"

#include <algorithm>

DiffdriveTeleop::DiffdriveTeleop(float maxLinearVelocity, float maxAngularVelocity)
    : m_maxLinearVelocity(maxLinearVelocity), m_maxAngularVelocity(maxAngularVelocity),
      m_noMessageReceivedCnt(0)
{
    m_velCommand.linear.x = 0;
    m_velCommand.linear.y = 0;
    m_velCommand.linear.z = 0;

    m_velCommand.angular.x = 0;
    m_velCommand.angular.y = 0;
    m_velCommand.angular.z = 0;

    m_velPublisher = m_nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    m_checkMessageTimer
        = m_nodeHandle.createTimer(ros::Duration(0.5), &DiffdriveTeleop::checkLastMessage, this);
}

void DiffdriveTeleop::setVelocity(float linearVelocity, float angularVelocity)
{
    if (linearVelocity > 0)
    {
        linearVelocity = std::min(linearVelocity, m_maxLinearVelocity);
    }
    else if (linearVelocity < 0)
    {
        linearVelocity = std::max(linearVelocity, -m_maxLinearVelocity);
    }

    if (angularVelocity > 0)
    {
        angularVelocity = std::min(angularVelocity, m_maxAngularVelocity);
    }
    else if (angularVelocity < 0)
    {
        angularVelocity = std::max(angularVelocity, -m_maxAngularVelocity);
    }

    m_velCommand.linear.x  = linearVelocity;
    m_velCommand.angular.z = angularVelocity;

    m_velPublisher.publish(m_velCommand);
    m_noMessageReceivedCnt = 0;
}

void checkLastMessage(const ros::TimerEvent& t_event)
{
    m_noMessageReceivedCnt++;
    if (m_noMessageReceivedCnt >= 2)
    {
        m_velCommand.linear.x  = 0;
        m_velCommand.angular.z = 0;

        m_velPublisher.publish(m_velCommand);
    }
}