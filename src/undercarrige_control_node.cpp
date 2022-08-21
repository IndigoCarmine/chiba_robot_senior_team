
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include "common_settings.hpp"


namespace undercarrige_control_node{
    //the class is a node to calculate undercarrige speed from two joystick.
    //joystick message is sent as geometry_msgs::Twist.
    //the right joystick is geometry_msgs::Twist.linear.x, geometry_msgs::Twist.linear.y
    //the left joystick is geometry_msgs::Twist.angular.x, geometry_msgs::Twist.angular.y
    //TODO : calibrate leftjoystick_sensitivity.
    class UndercarrigeControlNode: public nodelet::Nodelet {
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher pub_;
            const int leftstick_sensitivity_ = 10;
        public:
            void onInit(){
                NODELET_INFO("UndercarrigeControlNode is started.");
                nodehandle_ = getMTNodeHandle();
                sub_ = nodehandle_.subscribe<common_settings::topic::JoystickParams::Message>(common_settings::topic::JoystickParams::name, 1, &UndercarrigeControlNode::callback, this);
                pub_ = nodehandle_.advertise<common_settings::topic::UndercarriageParams::Message>(common_settings::topic::UndercarriageParams::name, 1);
            }

        private:
            //joystick message is sent as geometry_msgs::Twist.
            //the right joystick is geometry_msgs::Twist.linear.x, geometry_msgs::Twist.linear.y
            //the left joystick is geometry_msgs::Twist.angular.x, geometry_msgs::Twist.angular.y
            void callback(const geometry_msgs::TwistStamped::ConstPtr &data){
                if(data->header.frame_id ==common_settings::normal_mode){
                    geometry_msgs::Twist output;

                    output.linear.x = data->twist.linear.x;
                    output.linear.y = data->twist.linear.y + data->twist.angular.y * leftstick_sensitivity_;
                    output.angular.z = data->twist.angular.x;
                    pub_.publish(output);
                }else if(data->header.frame_id ==common_settings::aiming_mode){
                    geometry_msgs::Twist output;
                    output.linear.x = data->twist.linear.x;
                    output.linear.y = data->twist.linear.y;
                    pub_.publish(output);
                }else{
                    NODELET_WARN("UndercarrigeControlNode::  get wrong frame_id");
                }
            }
    };

}//namespace undercarrige_control_node
PLUGINLIB_EXPORT_CLASS(undercarrige_control_node::UndercarrigeControlNode, nodelet::Nodelet)