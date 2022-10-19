#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <can_utils_rev.hpp>
#include <can_plugins/Frame.h>
#include <array>
#include "common_settings.hpp"
using namespace common_settings;

namespace odometry_calculate_node{
    class OdometryCalculateNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher pub_;
            ros::Subscriber reset_sub_;
            std::array<double,2> body_position_ = {0,0};
            std::array<double,2> body_velocity_ = {0,0};
            double body_angle_ = 0;//degree
            double body_angular_velocity_ = 0;//degree
            ros::Time last_time_ = ros::Time::now();
            double error_duration_ = 0.1;

        public:

            void onInit(){
                nodehandle_ = getNodeHandle();
                sub_ = nodehandle_.subscribe<topic::OdometryParams::Message>(topic::OdometryParams::name, 1, &OdometryCalculateNode::callback, this);
                pub_ = nodehandle_.advertise<topic::UnitPositionParams::Message>(topic::UnitPositionParams::name, 1);
                reset_sub_ = nodehandle_.subscribe<topic::UnitPositionReset::Message>(topic::UnitPositionReset::name, 1, &OdometryCalculateNode::resetCallback, this);
                NODELET_INFO("OdometryTransformNode is started.");
            }

            void callback(const topic::OdometryParams::Message::ConstPtr &msg){
                //Message means the velocity of the robot. and 
                //message.x and message.y is the velocity on relative coordinate.
                //message.z is the angular velocity on universal coordinate.

                //calculate duration
                double duration = (ros::Time::now() - last_time_).toSec();
                //show error if duration is too long
                if(duration>=error_duration_) {
                    NODELET_ERROR("OdometryCalculateNode: duration is too long. ");
                }

                //Integrate the odometry message to get the position of the body.
                //The position of the body is the position of the center of the body. And it is a relative coordinates.
                //approximate the position of the body. 
                
                //angular velocity change linearly.
                body_angle_ += (body_angular_velocity_ + msg->angular.z)/2 * duration;

                //change linear velocity to one on universal coordinate.
                double linear_velocity_x = msg->linear.x * cos(body_angle_) - msg->linear.y * sin(body_angle_);
                double linear_velocity_y = msg->linear.x * sin(body_angle_) + msg->linear.y * cos(body_angle_);


                //linear velocity change linearly.
                body_position_[0] += (body_velocity_[0] + linear_velocity_x)/2 * duration;  
                body_position_[1] += (body_velocity_[1] + linear_velocity_y)/2 * duration;  

                //set last parameters
                body_velocity_[0] = linear_velocity_x;
                body_velocity_[1] = linear_velocity_y;
                body_angular_velocity_ = msg->angular.z;
                last_time_ = ros::Time::now();

                //publish the position of the body.
                topic::UnitPositionParams::Message position_msg;
                position_msg.x = body_position_[0];
                position_msg.y = body_position_[1];
                position_msg.z = body_angle_;
                pub_.publish(position_msg);
            }

            void resetCallback(const topic::UnitPositionReset::Message::ConstPtr &msg){
                //reset the position of the body.
                body_position_[0] = 0;
                body_position_[1] = 0;
                body_angle_ = 0;
                body_velocity_[0] = 0;
                body_velocity_[1] = 0;
                body_angular_velocity_ = 0;
            }

    };
} // namespace odometry_calculate_node
PLUGINLIB_EXPORT_CLASS(odometry_calculate_node::OdometryCalculateNode, nodelet::Nodelet)