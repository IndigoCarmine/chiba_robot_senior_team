#include "can_input_node.hpp"
#include <geometry_msgs/Vector3.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>



namespace undercarrige_transform_node{
    class UndercarrigeTransformNode : public can_input_node::CanInputNode<geometry_msgs::Vector3,geometry_msgs::Vector3::ConstPtr>{
        public:
            void onInit()override{
                topic_name_ = "undercarrige_velocity";
                CanInputNode::onInit();
            }
        private:
            void can_input(const geometry_msgs::Vector3::ConstPtr &data)override{
                //TODO 
                ROS_ASSERT("undercarrige_transform_node::transform is not implemented yet");
                can_plugins::Frame frame;
                publish(frame);
                
            }
    };
    
} // namespace undercarrige_transform_node
PLUGINLIB_EXPORT_CLASS(undercarrige_transform_node::UndercarrigeTransformNode, nodelet::Nodelet);