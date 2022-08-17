#include <ros/common.h>

#include "transform_node.hpp"
#include <geometry_msgs/Twist.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace actiuaters_transform_node{
    class ActiatersTransformNode : transform_node::TransformNode<geometry_msgs::Twist>{
        public:
        virtual void onInit(){
            topic_name_ = "actuater_parameters";
            TransformNode::onInit();
        };
        private:
        void transform(const geometry_msgs::Twist data){
            //TODO
            ROS_ASSERT("actuaters_transform_node::transform is not implemented yet");
            

        };
    };
} // namespace undercarrige_transform_node

PLUGINLIB_EXPORT_CLASS(actiuaters_transform_node::ActiatersTransformNode, nodelet::Nodelet);