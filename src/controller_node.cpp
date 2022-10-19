#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Joy.h>
#include "common_settings.hpp"
using namespace common_settings;

namespace controller_node{
    class ControllerNode : public nodelet::Nodelet{
        private:
            ros::Subscriber joy_sub_;
            ros::Publisher twist_pub_;
            ros::Publisher actuators_pub_;
            std::string joy_frame_id_ = frame_id::normal_mode;
            ros::NodeHandle nh_;
        public:
        void onInit(){
            nh_ = getNodeHandle();
            joy_sub_ = nh_.subscribe("joy", 1, &ControllerNode::joyCallback, this);
            twist_pub_ = nh_.advertise<topic::JoystickParams::Message>(topic::JoystickParams::name, 1);
            actuators_pub_ = nh_.advertise<std_msgs::Int32>("", 1);
            NODELET_INFO("controller_node has started.");
            
        }
        private:

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
            //if push a button, change the frame_id to aiming_mode
            if(msg->buttons[1] == 1){
                joy_frame_id_ = frame_id::aiming_mode;
            }
            //if push b button, change the frame_id to normal_mode
            if(msg->buttons[2] == 1){
                joy_frame_id_ = frame_id::normal_mode;
            }
            //if push y button, change the frame_id to auto_mode
            if(msg->buttons[3] == 1){
                joy_frame_id_ = frame_id::auto_mode;
            }
            
            //publish joy_params to undercarriage control and 
            //the right joystick is geometry_msgs::Twist.linear.x, geometry_msgs::Twist.linear.y
            //the left joystick is geometry_msgs::Twist.angular.x, geometry_msgs::Twist.angular.y
            topic::JoystickParams::Message twiststmped;
            twiststmped.header.frame_id = joy_frame_id_;
            twiststmped.twist.linear.x = msg->axes[0];
            twiststmped.twist.linear.y = msg->axes[1];
            twiststmped.twist.angular.x= msg->axes[2];
            twiststmped.twist.angular.y= msg->axes[3];
            twist_pub_.publish(twiststmped);

            //publish button message
            //TODO
            NODELET_WARN("You should write joyCallback in ControllerNode.");

            //actuators_pub_.publish(something);

        }
    };
} // namespace controller_node
PLUGINLIB_EXPORT_CLASS(controller_node::ControllerNode, nodelet::Nodelet)