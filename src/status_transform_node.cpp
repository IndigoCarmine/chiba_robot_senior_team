#include "can_output_node.hpp"
#include <std_msgs/Int32.h>

namespace status_transform_node{
    class StatusTransformNode : public can_output_node::CanOutputNode<std_msgs::Int32>{
        public:
            StatusTransformNode(){
                topic_name_ = "status_tx";
            };
            virtual can_plugins::Frame transform(const std_msgs::Int32 data);
    };
}