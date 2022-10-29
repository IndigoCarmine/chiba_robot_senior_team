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
#include <chiba_robot_senior_team/ParameterNotice.h>
#include <chiba_robot_senior_team/ParameterService.h>
#include <chiba_robot_senior_team/IDService.h>
#include <chiba_robot_senior_team/Turret.h>
#include <chiba_robot_senior_team/SensorParameter.h>
#include <chiba_robot_senior_team/Work.h>
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
        CHIBAROBO_deine_topic(TurretParams,turret_params,chiba_robot_senior_team::Turret);// from TurretControlNode to TurretTransformNode. it means a direction of the turret wheel.
        CHIBAROBO_deine_topic(StatusParams, status_params, std_msgs::Int32); //from StatusTransformNode to OtherNodes (Websocket). it means a status of each motor.
        CHIBAROBO_deine_topic(JoystickParams, joystick_params, geometry_msgs::TwistStamped);//from ControllerNode to UndercarriageControlNode and TurretWheelControlNode. it means a value of two joysticks.
        CHIBAROBO_deine_topic(OdometryParams, odometry_params, geometry_msgs::Twist);// from OdometryTransformNode to OdometryCaluculateNode. it means a value of odometry.
        CHIBAROBO_deine_topic(UnitPositionParams, unit_position_params, geometry_msgs::Vector3);// from OdometryCalculateNode to ??  it means a position of the body.
        CHIBAROBO_deine_topic(EmergencyCmd, emergency_cmd, std_msgs::Bool);//from ?? to EmergencyTransformNode. it means a emergency cmd and restary cmd.( it may become other type.)
        CHIBAROBO_deine_topic(UnitPositionReset, unit_position_reset, std_msgs::Bool); // from ?? to OdometryCalculateNode. it means a reset cmd of the position of the body.
        CHIBAROBO_deine_topic(SensorParams, sensor_params, chiba_robot_senior_team::SensorParameter);// from SensorTransformNode to SensorCaluculateNode. it means a value of sensor.
        CHIBAROBO_deine_topic(Work, work, chiba_robot_senior_team::Work);// from WorkConductorNode to ??. it means a work.
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
        aiming_mode = "aiming_mode",
        auto_mode = "auto_mode"
        ;
    } // namespace frame_id
    
} // namespace common_settings