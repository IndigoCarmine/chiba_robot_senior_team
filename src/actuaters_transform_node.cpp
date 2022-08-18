#include "can_input_node.hpp"
#include <geometry_msgs/Twist.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace actiuaters_transform_node{
    class ActiatersTransformNode : public can_input_node::CanInputNode<geometry_msgs::Twist, geometry_msgs::Twist::ConstPtr>{
        public:
        void onInit()override{
            topic_name_ = "actuater_parameters";
            callback_ = [this](const geometry_msgs::Twist::ConstPtr& msg){this->actuatersCallback(msg);};
            CanInputNode::onInit();
        }
        protected:
        void actuatersCallback(const geometry_msgs::Twist::ConstPtr &data){
            //TODO
            ROS_ASSERT("actuaters_transform_node::actuatersCallback is not implemented yet");
            

        }
    };
} // namespace undercarrige_transform_node

PLUGINLIB_EXPORT_CLASS(actiuaters_transform_node::ActiatersTransformNode, nodelet::Nodelet);