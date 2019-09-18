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

    ros::NodeHandle nhPrivate("~");

    std::string port;
    nhPrivate.param("port", port, std::string("/dev/ttyACM0"));
    int baudRate;
    nhPrivate.param("baud_rate", baudRate, 115200);
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

    AckermannToDiffdriveTeleopSiaUart teleop(port,
                                             baudRate,
                                             maxSteeringWheelAngle,
                                             maxThrottleState,
                                             maxBrakeState,
                                             maxAcceleration,
                                             defaultDeceleration,
                                             maxBrakeState,
                                             maxLinearVelocity,
                                             maxAngularVelocity,
                                             numGears);

    ros::spin();
}