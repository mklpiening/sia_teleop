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

    AckermannToDiffdriveTeleopSiaUdp teleop(1337, 90.0, 100.0, 100.0, 1.5, 0.4, 2, 1, 1, 3);

    ros::spin();
}