#define TOPIC_NAME "status_params"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <can_plugins/Frame.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

namespace status_transform_node{
    class StatusTransformNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Publisher pub_;
            ros::Subscriber can_rx_sub_;
        public:
            void onInit(){
                nodehandle_ = getNodeHandle();
                can_rx_sub_ = nodehandle_.subscribe("can_rx", 1000,&StatusTransformNode::callback, this);
                pub_ = nodehandle_.advertise<std_msgs::Int32>(TOPIC_NAME, 1000);
                NODELET_INFO("StatusTransformNode is started.");
            }
            void callback(const can_plugins::Frame::ConstPtr &msg){
                //TODO
                NODELET_WARN("status_transform_node::statusCallback is not implemented yet");
            }
    };
}
PLUGINLIB_EXPORT_CLASS(status_transform_node::StatusTransformNode, nodelet::Nodelet);