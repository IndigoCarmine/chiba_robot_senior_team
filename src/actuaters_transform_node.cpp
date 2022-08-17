#include "can_input_node.hpp"
#include <geometry_msgs/Twist.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace actiuaters_transform_node{
    class ActiatersTransformNode : public can_input_node::CanInputNode<geometry_msgs::Twist, geometry_msgs::Twist::ConstPtr>{
        public:
        void onInit()override{
            topic_name_ = "actuater_parameters";
            CanInputNode::onInit();
        }
        protected:
        void can_input(const geometry_msgs::Twist::ConstPtr &data)override{
            //TODO
            ROS_ASSERT("actuaters_transform_node::transform is not implemented yet");
            

        }
    };
} // namespace undercarrige_transform_node

PLUGINLIB_EXPORT_CLASS(actiuaters_transform_node::ActiatersTransformNode, nodelet::Nodelet);