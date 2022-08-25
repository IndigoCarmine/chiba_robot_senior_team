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
            nodehandle_ = getNodeHandle();
            can_tx_pub_ = nodehandle_.advertise<topic::CanTx::Message>(topic::CanTx::name, 1);
            sub_ = nodehandle_.subscribe<topic::TurretWheelParams::Message>(topic::TurretWheelParams::name, 1, &ActuatorsTransformNode::callback, this);
        }
        protected:
        void callback(const topic::TurretWheelParams::Message::ConstPtr &data){
            //TODO
            NODELET_WARN("actuaters_transform_node::actuatersCallback is not implemented yet");
            
            can_plugins::Frame frame;
            can_tx_pub_.publish(frame);

        }
    };
} // namespace undercarrige_transform_node

PLUGINLIB_EXPORT_CLASS(actuators_transform_node::ActuatorsTransformNode, nodelet::Nodelet)