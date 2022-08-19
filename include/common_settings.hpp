#include <string>

namespace common_settings{

    //It is TopicName
    //I understand that this is not a good way to do this, but I am not sure how to do it better.
    //If you hate this way, you should hate cpp, which doesn't have enum class with string. OR YOU SHOULD change it.
    const std::string 
        can_rx = "can_rx",
        can_tx = "can_tx",
        undercarrige_param = "undercarrige_params",
        turret_wheel_params = "turret_wheel_params",
        elevation_angle = "elevation_angle",
        shooting_speed = "shooting_speed",
        status_param = "status_params",
        joystick_params = "joystick_params",
        odometry_param = "odometry_param",
        emergency_cmd = "emergency_cmd"
    ;

    //It is FrameId
    const std::string 
        normal_mode = "normal_mode",
        aiming_mode = "aiming_mode"
    ;


} // namespace common_settings