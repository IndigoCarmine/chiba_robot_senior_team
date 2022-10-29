
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include "common_settings.hpp"
using namespace common_settings;

namespace undercarrige_control_node{
    //the class is a node to calculate undercarrige speed from two joystick.
    //joystick message is sent as geometry_msgs::Twist.
    //the right joystick is geometry_msgs::Twist.linear.x, geometry_msgs::Twist.linear.y
    //the left joystick is geometry_msgs::Twist.angular.x, geometry_msgs::Twist.angular.y
    //TODO : calibrate leftjoystick_sensitivity.
    class UndercarrigeControlNode: public nodelet::Nodelet {
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber joystick_sub_;
            ros::Subscriber odometry_sub_;
            ros::Publisher pub_;
            int leftstick_sensitivity_ = 10;


            //for parameter_server
            ros::ServiceClient service_client_;
            ros::Subscriber notify_sub_;
        public:
            void onInit(){
                nodehandle_ = getNodeHandle();
                joystick_sub_ = nodehandle_.subscribe<topic::JoystickParams::Message>(topic::JoystickParams::name, 1, &UndercarrigeControlNode::joystickCallback, this);
                pub_ = nodehandle_.advertise<topic::UndercarriageParams::Message>(topic::UndercarriageParams::name, 1);
                service_client_ = nodehandle_.serviceClient<topic::GetParameter::Message>(topic::GetParameter::name);
                notify_sub_ = nodehandle_.subscribe(topic::ParameterChangeNotify::name,10,&UndercarrigeControlNode::changeParameter,this);
                odometry_sub_ = nodehandle_.subscribe(topic::OdometryParams::name,10,&UndercarrigeControlNode::odometryCallback,this);
                NODELET_INFO("UndercarrigeControlNode is started.");
            }

        private:
            void changeParameter(const topic::ParameterChangeNotify::Message::ConstPtr &notice){
                
                if(notice->parameter_name == "leftstick_sensitivity") leftstick_sensitivity_ = notice->parameter;
                
            }


            //joystick message is sent as geometry_msgs::Twist.
            //the right joystick is geometry_msgs::Twist.linear.x, geometry_msgs::Twist.linear.y
            //the left joystick is geometry_msgs::Twist.angular.x, geometry_msgs::Twist.angular.y
            void joystickCallback(const topic::JoystickParams::Message::ConstPtr &data){
                if(data->header.frame_id ==frame_id::normal_mode){
                    geometry_msgs::Twist output;

                    output.linear.x = data->twist.angular.x;
                    output.linear.y = data->twist.angular.y + data->twist.linear.y * leftstick_sensitivity_;
                    output.angular.z = data->twist.linear.x;
                    pub_.publish(output);
                }else if(data->header.frame_id ==frame_id::aiming_mode){
                    geometry_msgs::Twist output;
                    output.linear.x = data->twist.angular.x;
                    output.linear.y = data->twist.angular.y;
                    pub_.publish(output);
                }else{
                    NODELET_WARN("UndercarrigeControlNode::  get wrong frame_id");
                }
            }


            void odometryCallback(const topic::OdometryParams::Message::ConstPtr &data){
            }
    };

}//namespace undercarrige_control_node
PLUGINLIB_EXPORT_CLASS(undercarrige_control_node::UndercarrigeControlNode, nodelet::Nodelet)