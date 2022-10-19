#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <can_utils_rev.hpp>

#include <can_plugins/Frame.h>
#include <geometry_msgs/Twist.h>

#include "common_settings.hpp"
using namespace common_settings;

namespace actuators_transform_node {
    class ActuatorsTransformNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher can_tx_pub_;
        public:
        void onInit()override{
        }
    };
} // namespace undercarrige_transform_node

PLUGINLIB_EXPORT_CLASS(actuators_transform_node::ActuatorsTransformNode, nodelet::Nodelet)