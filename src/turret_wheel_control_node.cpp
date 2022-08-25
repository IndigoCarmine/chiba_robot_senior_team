#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/TwistStamped.h>
#include <can_utils_rev.hpp>
#include <can_plugins/Frame.h>
#include "common_settings.hpp"
using namespace common_settings;

namespace turret_wheel_control_node{
    class TurretWheelControlNode: public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher pub_;
        public:
            void onInit(){
                nodehandle_ = getNodeHandle();
                sub_ = nodehandle_.subscribe<topic::JoystickParams::Message>(topic::JoystickParams::name, 1, &TurretWheelControlNode::callback, this);
                pub_ = nodehandle_.advertise<topic::TurretWheelParams::Message>(topic::TurretWheelParams::name, 1);
                NODELET_INFO("TurretWheelControlNode is started.");
            }
            protected:
            void callback(const geometry_msgs::TwistStamped::ConstPtr &data){
                //TODO
                NODELET_WARN("TurretWheelControlNode::callback is not implemented yet");
                //joystick message is sent as geometry_msgs::Twist.
                //the right joystick is geometry_msgs::Twist.linear.x, geometry_msgs::Twist.linear.y
                //the left joystick is geometry_msgs::Twist.angular.x, geometry_msgs::Twist.angular.y
                if(data->header.frame_id == frame_id::aiming_mode){
                    pub_.publish(data->twist.angular);
                }

            }
    };
}//namespace turret_wheel_control_node
PLUGINLIB_EXPORT_CLASS(turret_wheel_control_node::TurretWheelControlNode, nodelet::Nodelet)