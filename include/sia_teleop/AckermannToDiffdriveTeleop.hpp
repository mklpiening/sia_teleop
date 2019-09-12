#ifndef ACKERMANN_TO_DIFFDRIVE_TELEOP_HPP
#define ACKERMANN_TO_DIFFDRIVE_TELEOP_HPP

#include "sia_teleop/DiffdriveTeleop.hpp"

/**
 * @brief receives controls as ackermann kinematics and publishes velocity commands for diffdrive
 * vehicle
 */
class AckermannToDiffdriveTeleop : public DiffdriveTeleop
{
  public:
    /**
     * @brief Construct a new Ackermann To Diffdrive Teleop object
     * 
     * @param maxAcceleration acceleration at its peak in (intensity / s)
     * @param defaultDeceleration deceleration when rolling
     * @param maxBrakeDeceleration deceleration whith maximum brake state 
     * @param maxLinearVelocity maximum linear velocity
     * @param maxAngularVelocity maximum angular velocity
     */
    AckermannToDiffdriveTeleop(float maxAcceleration      = 0.4,
                               float defaultDeceleration  = 0.1,
                               float maxBrakeDeceleration = 1.0,
                               float maxLinearVelocity    = 1.0,
                               float maxAngularVelocity   = 1.0);
    /**
     * @brief Set the current steeringwheel angle and publishes the resulting velocities
     *
     * @param angle angle of the steering wheel in a range of -1 to 1 where zero is the neutral
     * position
     */
    void setSteeringWheelAngle(float angle);

    /**
     * @brief Set the throttle and publshes the resulting velocities
     *
     * @param throttle state of throttle in range of -1 to 1 where -1 is full speed reverse and 1 is
     * full speed forward
     */
    void setThrottle(float throttle);

    /**
     * @brief brakes the vehicle
     *
     * @param brake state of brake between 0 and 1 where 0 is no braking and 1 is full braking
     */
    void setBrake(float brake);

  private:
    /**
     * @brief publshes the diffdrive velocities according to the current steeringwheel angle and the
     * throttle state
     */
    void updateDiffdriveVelocity(const ros::TimerEvent& t_event);

    ros::Timer m_updateMovementTimer;

    float m_defaultDeceleration;
    float m_maxBrakeDeceleration;
    float m_maxAcceleration;

    float m_speed;

    float m_steeringAngle;
    float m_throttle;
    float m_brake;
};

#endif