#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <can_utils_rev.hpp>
#include <can_plugins/Frame.h>
#include "common_settings.hpp"

namespace odometry_transform_node{
    class OdometryTransformNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber can_rx_sub_;
            ros::Publisher pub_;
        public:
            void onInit(){
                nodehandle_ = getMTNodeHandle();
                can_rx_sub_ = nodehandle_.subscribe<common_settings::topic::CanRx::Message>(common_settings::topic::CanRx::name, 1, &OdometryTransformNode::callback, this);
                pub_ = nodehandle_.advertise<common_settings::topic::OdometryParams::Message>(common_settings::topic::OdometryParams::name, 1);
                NODELET_INFO("OdometryTransformNode is started.");  
            }
        private:
            void callback(const can_plugins::Frame::ConstPtr &data){
                NODELET_WARN("OdometryTransformNode::callback is not implemented yet");
            }
    };
} // namespace odometry_transform_node
PLUGINLIB_EXPORT_CLASS(odometry_transform_node::OdometryTransformNode, nodelet::Nodelet)