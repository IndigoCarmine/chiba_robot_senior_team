#include "can_output_node.hpp"
#include <std_msgs/Int32.h>
#include <ros/ros.h>

namespace status_transform_node{
    class StatusTransformNode : public can_output_node::CanOutputNode<std_msgs::Int32>{
        public:
            void onInit()override{
                topic_name_ = "status";
                CanOutputNode::onInit();
            }
            void can_output(const can_plugins::Frame::ConstPtr &msg)override{
                //TODO
                ROS_ASSERT("status_transform_node::can_output is not implemented yet");
            }
    };
}