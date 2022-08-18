#include "can_input_node.hpp"
#include <geometry_msgs/Vector3.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>



namespace undercarrige_transform_node{
    class UndercarrigeTransformNode : public can_input_node::CanInputNode<geometry_msgs::Vector3,geometry_msgs::Vector3::ConstPtr>{
        public:
            void onInit()override{
                topic_name_ = "un_vel";
//                callback_  = boost::bind(callback_);
                callback_ = boost::bind(&UndercarrigeTransformNode::undercarrigeCallback,this,_1);
                CanInputNode::onInit();
            }
        private:
            void undercarrigeCallback(const geometry_msgs::Vector3::ConstPtr &data){
                //TODO 
                ROS_ASSERT("undercarrige_transform_node::undercarrigeCallback is not implemented yet");
                can_plugins::Frame frame;
                publish(frame);
                
            }
    };
    
} // namespace undercarrige_transform_node
PLUGINLIB_EXPORT_CLASS(undercarrige_transform_node::UndercarrigeTransformNode, nodelet::Nodelet);