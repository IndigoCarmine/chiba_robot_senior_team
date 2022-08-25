
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <can_plugins/Frame.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

#include <can_utils_rev.hpp>
#include "common_settings.hpp"
using namespace common_settings;

namespace status_transform_node{
    //The class is for transforming the status message from can message to ros message.
    class StatusTransformNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Publisher pub_;
            ros::Subscriber can_rx_sub_;
        public:
            void onInit(){
                nodehandle_ = getNodeHandle();
                can_rx_sub_ = nodehandle_.subscribe<topic::CanRx::Message>(topic::CanRx::name, 1,&StatusTransformNode::callback, this);
                pub_ = nodehandle_.advertise<topic::StatusParams::Message>(topic::StatusParams::name, 1);
                NODELET_INFO("StatusTransformNode is started.");
            }
            void callback(const can_plugins::Frame::ConstPtr &msg){
                //TODO
                NODELET_WARN("status_transform_node::statusCallback is not implemented yet");
                //we have never difined shirasu status message.
            }
    };
}
PLUGINLIB_EXPORT_CLASS(status_transform_node::StatusTransformNode, nodelet::Nodelet)