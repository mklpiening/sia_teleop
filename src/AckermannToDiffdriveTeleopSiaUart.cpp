#include "sia_teleop/AckermannToDiffdriveTeleopSiaUart.hpp"

#include <sialib/SialibUart.hpp>

AckermannToDiffdriveTeleopSiaUart::AckermannToDiffdriveTeleopSiaUart(std::string port,
                                                                     int baudRate,
                                                                     float maxSteeringWheelAngle,
                                                                     float maxThrottleState,
                                                                     float maxBrakeState,
                                                                     float maxAcceleration,
                                                                     float defaultDeceleration,
                                                                     float maxBrakeDeceleration,
                                                                     float maxLinearVelocity,
                                                                     float maxAngularVelocity,
                                                                     int numGears)
    : AckermannToDiffdriveTeleopSia(std::make_shared<sialib::SialibUart>(port, baudRate),
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
    ros::init(argc, argv, "sia_diffdrive_uart");

    AckermannToDiffdriveTeleopSiaUart teleop(
        "/dev/ttyACM0", 115200, 90.0, 100.0, 100.0, 1.5, 0.4, 2, 1, 1, 3);

    ros::spin();
}