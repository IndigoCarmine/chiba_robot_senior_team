#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <can_utils_rev.hpp>
#include <can_plugins/Frame.h>
#include <array>
#include "common_settings.hpp"
using namespace common_settings;

namespace odometry_transform_node{
    class OdometryTransformNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber can_rx_sub_;
            ros::Publisher pub_;
            //TODO : Set the parameter
            uint32_t odometry_x_ = 0x001;
            uint32_t odometry_y_ = 0x002;
            uint32_t odometry_z_ = 0x003;

            std::array<uint8_t,3> odometry_data_;
            std::array<bool,3> odometry_is_set_;
        public:

            void onInit(){
                nodehandle_ = getNodeHandle();
                
                can_rx_sub_ = nodehandle_.subscribe<topic::CanRx::Message>(topic::CanRx::name, 1, &OdometryTransformNode::callback, this);
                pub_ = nodehandle_.advertise<topic::OdometryParams::Message>(topic::OdometryParams::name, 1);
                NODELET_INFO("OdometryTransformNode is started.");  
                NODELET_WARN("Message ID is not set.");
            }
        private:
            void callback(const can_plugins::Frame::ConstPtr &data){
                //Get these data from CAN, and publish them as geometry_msgs::Twist when all data is set.
                if(data->id == odometry_x_){
                    odometry_data_[0] = can_utils::unpack<int>(data->data);
                    odometry_is_set_[0] = true;
                }
                if(data->id == odometry_y_){
                    odometry_data_[1] = can_utils::unpack<int>(data->data);
                    odometry_is_set_[1] = true;
                }
                if(data->id == odometry_z_){
                    odometry_data_[2] = can_utils::unpack<int>(data->data);
                    odometry_is_set_[2] = true;
                }
                if(odometry_is_set_[0] && odometry_is_set_[1] && odometry_is_set_[2]){
                    topic::OdometryParams::Message output;
                    output.linear.x = odometry_data_[0];
                    output.linear.y = odometry_data_[1];
                    output.angular.z = odometry_data_[2];
                    pub_.publish(output);
                    odometry_is_set_[0] = false;
                    odometry_is_set_[1] = false;
                    odometry_is_set_[2] = false;
                }

            }
    };
} // namespace odometry_transform_node
PLUGINLIB_EXPORT_CLASS(odometry_transform_node::OdometryTransformNode, nodelet::Nodelet)