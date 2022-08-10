#include <ros/common.h>

#include "transform_node.hpp"
#include <geometry_msgs/Twist.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace actiuaters_transform_node{
    template<typename T>
    class ActiatersTransformNode : transform_node::TransformNode<T>{
        public:
        virtual void onInit(){
            topic_name_ = "actuater_parameters";
            TransformNode::onInit();
        };
        private:
        can_plugins::Frame transform(const geometry_msgs::Twist data){
            //TODO
            ROS_ASSERT("actuaters_transform_node::transform is not implemented yet");

            can_plugins::Frame msg;
            msg.id = 1;
            msg.data = {data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z};
            return msg;
        };
    };
    
} // namespace undercarrige_transform_node

PLUGINLIB_EXPORT_CLASS(actiuaters_transform_node::ActiatersTransformNode<geometry_msgs::Twist>, nodelet::Nodelet);