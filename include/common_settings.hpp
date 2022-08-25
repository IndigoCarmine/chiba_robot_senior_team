#include <can_plugins/Frame.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <chiba_robot_senior_team/IDNotice.h>
#include <chiba_robot_senior_team/ParameterService.h>
#include <chiba_robot_senior_team/ParameterNotice.h>
#include <chiba_robot_senior_team/IDService.h>
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
        CHIBAROBO_deine_topic(CanTx, can_tx, can_plugins::Frame);//can_tx
        CHIBAROBO_deine_topic(CanRx, can_rx, can_plugins::Frame);//can_rx
        CHIBAROBO_deine_topic(UndercarriageParams, undercarriage_params, geometry_msgs::Twist);// from UndercarriageControlNode to UndercarrigeTransformNode. it means a verocity of the body.
        CHIBAROBO_deine_topic(TurretWheelParams, turret_wheel_params, geometry_msgs::Vector3);// from TurretWheelControlNode to TurretWheelTransformNode. it means a direction of the turret wheel.
        CHIBAROBO_deine_topic(ElevationAngle, elevation_angle, std_msgs::Float32);// from ?? to ??. it means a elevation angle of the muzzle.
        CHIBAROBO_deine_topic(ShootingSpeed, shooting_speed, std_msgs::Float64); // from ?? to ??. it means a shooting speed (verocity of moters).
        CHIBAROBO_deine_topic(StatusParams, status_params, std_msgs::Int32); //from StatusTransformNode to OtherNodes (Websocket). it means a status of each motor.
        CHIBAROBO_deine_topic(JoystickParams, joystick_params, geometry_msgs::TwistStamped);//from ControllerNode to UndercarriageControlNode and TurretWheelControlNode. it means a value of two joysticks.
        CHIBAROBO_deine_topic(OdometryParams, odometry_params, geometry_msgs::Twist);// from OdometryTransformNode to ??. it means a position of the body.
        CHIBAROBO_deine_topic(EmergencyCmd, emergency_cmd, std_msgs::Bool);//from ?? to EmergencyTransformNode. it means a emergency cmd and restary cmd.( it may become other type.)
        
        //parameter_server
        CHIBAROBO_deine_topic(SetParameter,set_parameter, chiba_robot_senior_team::ParameterNotice);//from ?? to ParameterServerNode. it can set or change parameter.
        CHIBAROBO_deine_topic(ParameterChangeNotify, parameter_change_notify, chiba_robot_senior_team::ParameterNotice);// from ParameterServerNode to ??. it can notify that parameter is changed.
        CHIBAROBO_deine_topic(GetParameter, get_parameter, chiba_robot_senior_team::ParameterService);// from ?? to ParameterServerNode. it can get parameter.
        
        //id_manager
        CHIBAROBO_deine_topic(BindID,bind_id, chiba_robot_senior_team::IDNotice);//from ?? to IDManagerNode. it can set or change CAN ID.
        CHIBAROBO_deine_topic(IDChangeNotify, id_change_notify, chiba_robot_senior_team::IDNotice);// from IDManagerNode to ??. it can notify that CAN ID is changed.
        CHIBAROBO_deine_topic(GetID, get_id, chiba_robot_senior_team::IDService);// from ?? to IDManagerNode. it can get CAN ID.
    }

#undef CHIBAROBO_deine_topic
    
    namespace frame_id
    {
        //it use in joystic_params.
        const std::string 
        normal_mode = "normal_mode",
        aiming_mode = "aiming_mode"
        ;
    } // namespace frame_id
    
} // namespace common_settings