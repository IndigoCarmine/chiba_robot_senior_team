#include <can_plugins/Frame.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

namespace common_settings{

#define CHIBAROBO_deine_topic(TopicType, topic_name, MessageType)\
struct TopicType{\
    static constexpr const char name[] = #topic_name;\
    using Message = MessageType;\
}

    // This is Topic.
    // Since c++17, static constexpr variable is inplicitly inlined.
    // And it is deprecated to define static constexpr variable redundantly in source file.
    namespace topic
    {
        CHIBAROBO_deine_topic(CanTx, can_tx, can_plugins::Frame);
        CHIBAROBO_deine_topic(CanRx, can_rx, can_plugins::Frame);
        CHIBAROBO_deine_topic(UndercarriageParams, undercarriage_params, geometry_msgs::Twist);
        CHIBAROBO_deine_topic(TurretWheelParams, turret_wheel_params, geometry_msgs::Vector3);
        CHIBAROBO_deine_topic(ElevationAngle, elevation_angle, std_msgs::Float32);
        CHIBAROBO_deine_topic(ShootingSpeed, shooting_speed, std_msgs::Float64);
        CHIBAROBO_deine_topic(StatusParams, status_params, std_msgs::Int32);
        CHIBAROBO_deine_topic(JoystickParams, joystick_params, geometry_msgs::TwistStamped);
        CHIBAROBO_deine_topic(OdometryParams, odometry_params, geometry_msgs::Twist);
        CHIBAROBO_deine_topic(EmergencyCmd, emergency_cmd, std_msgs::Bool);
    }

#undef CHIBAROBO_deine_topic

    //It is FrameId
    const std::string 
        normal_mode = "normal_mode",
        aiming_mode = "aiming_mode"
    ;


} // namespace common_settings