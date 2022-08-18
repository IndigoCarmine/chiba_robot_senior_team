#include "can_output_node.hpp"
#include <std_msgs/Int32.h>
#include <ros/ros.h>

namespace status_transform_node{
    class StatusTransformNode : public can_output_node::CanOutputNode<std_msgs::Int32>{
        public:
            void onInit()override{
                topic_name_ = "status";
                callback_ = [this](const can_plugins::Frame::ConstPtr& msg){this->statusCallback(msg);};
                CanOutputNode::onInit();
            }
            void statusCallback(const can_plugins::Frame::ConstPtr &msg){
                //TODO
                ROS_ASSERT("status_transform_node::statusCallback is not implemented yet");
            }
    };
}