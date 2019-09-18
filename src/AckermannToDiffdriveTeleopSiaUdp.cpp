#include "sia_teleop/AckermannToDiffdriveTeleopSiaUdp.hpp"

#include <sialib/SialibUdp.hpp>

AckermannToDiffdriveTeleopSiaUdp::AckermannToDiffdriveTeleopSiaUdp(int port,
                                                                   float maxSteeringWheelAngle,
                                                                   float maxThrottleState,
                                                                   float maxBrakeState,
                                                                   float maxAcceleration,
                                                                   float defaultDeceleration,
                                                                   float maxBrakeDeceleration,
                                                                   float maxLinearVelocity,
                                                                   float maxAngularVelocity,
                                                                   int numGears)
    : AckermannToDiffdriveTeleopSia(std::make_shared<sialib::SialibUdp>(port),
                                    maxSteeringWheelAngle,
                                    maxThrottleState,
                                    maxBrakeState,
                                    maxAcceleration,
                                    defaultDeceleration,
                                    maxBrakeDeceleration,
                                    maxLinearVelocity,
                                    maxAngularVelocity,
                                    numGears)
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sia_diffdrive_udp");

    ros::NodeHandle nhPrivate("~");
    
    int port;
    nhPrivate.param("port", port, 1337);
    float maxSteeringWheelAngle;
    nhPrivate.param("max_steering_angle", maxSteeringWheelAngle, 90.0f);
    float maxThrottleState;
    nhPrivate.param("max_throttle_state", maxThrottleState, 100.0f);
    float maxBrakeState;
    nhPrivate.param("max_brake_state", maxBrakeState, 100.0f);
    float maxAcceleration;
    nhPrivate.param("max_acceleration", maxAcceleration, 1.5f);
    float defaultDeceleration;
    nhPrivate.param("default_deceleration", defaultDeceleration, 0.4f);
    float maxBrakeDeceleration;
    nhPrivate.param("max_brake_deceleration", maxBrakeDeceleration, 2.0f);
    float maxLinearVelocity;
    nhPrivate.param("max_linear_velocity", maxLinearVelocity, 1.0f);
    float maxAngularVelocity;
    nhPrivate.param("max_angular_velocity", maxAngularVelocity, 1.0f);
    int numGears;
    nhPrivate.param("num_gears", numGears, 3);

    AckermannToDiffdriveTeleopSiaUdp teleop(port,
                                            maxSteeringWheelAngle,
                                            maxThrottleState,
                                            maxBrakeState,
                                            maxAcceleration,
                                            defaultDeceleration,
                                            maxBrakeDeceleration,
                                            maxLinearVelocity,
                                            maxAngularVelocity,
                                            numGears);

    ros::spin();
}