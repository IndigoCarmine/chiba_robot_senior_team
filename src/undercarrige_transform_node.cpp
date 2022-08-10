#include <ros/common.h>

#include "transform_node.hpp"
#include <geometry_msgs/Twist.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace undercarrige_transform_node{
    template<typename T>
    class UndercarrigeTransformNode : transform_node::TransformNode<T>{
        public:
        virtual void onInit(){
            topic_name_ = "undercarrige_velocity";
            TransformNode::onInit();
        };
        private:
        can_plugins::Frame transform(const geometry_msgs::Twist data){
            //TODO 
            ROS_ASSERT("undercarrige_transform_node::transform is not implemented yet");
            
            can_plugins::Frame msg;
            msg.id = 1;
            msg.data = {data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z};
            return msg;
        };
    };
    
} // namespace undercarrige_transform_node

PLUGINLIB_EXPORT_CLASS(undercarrige_transform_node::UndercarrigeTransformNode<geometry_msgs::Twist>, nodelet::Nodelet);