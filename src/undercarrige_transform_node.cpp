#include <ros/common.h>

#include "transform_node.hpp"
#include <geometry_msgs/Vector3.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <vector>


namespace undercarrige_transform_node{
    class UndercarrigeTransformNode : transform_node::TransformNode<geometry_msgs::Vector3>{
        public:
        virtual void onInit(){
            topic_name_ = "undercarrige_velocity";
            TransformNode::onInit();
        };
        private:

        void transform(const geometry_msgs::Vector3 data){
            //TODO 
            ROS_ASSERT("undercarrige_transform_node::transform is not implemented yet");
            can_plugins::Frame frame;
        
            publish(frame)
            
        };
    };
    
} // namespace undercarrige_transform_node

PLUGINLIB_EXPORT_CLASS(undercarrige_transform_node::UndercarrigeTransformNode, nodelet::Nodelet);