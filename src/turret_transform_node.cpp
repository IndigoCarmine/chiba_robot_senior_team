
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Vector3.h>
#include <can_utils_rev.hpp>
#include <can_plugins/Frame.h>
#include <common_settings.hpp>

namespace turret_transform_node{
    
    class TurretTransformNode: public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher can_tx_pub_;
            uint16_t evaluation_angle_id_;
            uint16_t speed_upper_id = 0x000;
            uint16_t speed_lower_id = 0x000;
            uint16_t speed_left_id  = 0x000;
            uint16_t speed_right_id = 0x000;
        public:
            void onInit(){
                nodehandle_ = getNodeHandle();
                can_tx_pub_ = nodehandle_.advertise<common_settings::topic::CanTx::Message>(common_settings::topic::CanTx::name, 1);
                sub_ = nodehandle_.subscribe<common_settings::topic::TurretParams::Message>(common_settings::topic::TurretParams::name, 1, &TurretTransformNode::callback, this);
                //TODO: get id from parameter server, OR set right id. IT IS TEST PARAMETER.

                NODELET_INFO("TurretWheelTransformNode is initialized");
                
                NODELET_WARN("TurretWheelTransformNode : id is not implemented yet");
            }

            protected:
            void callback(const common_settings::topic::TurretParams::Message::ConstPtr &data){
                //make can frame and publish it.
                can_tx_pub_.publish(can_utils::makeFrame(evaluation_angle_id_, data->evaluation_angle));
                can_tx_pub_.publish(can_utils::makeFrame(speed_upper_id, data->speed_upper));
                can_tx_pub_.publish(can_utils::makeFrame(speed_lower_id, data->speed_lower));
                can_tx_pub_.publish(can_utils::makeFrame(speed_left_id, data->speed_left));
                can_tx_pub_.publish(can_utils::makeFrame(speed_right_id, data->speed_right));   
            }
    };

}//namespace turret_wheel_transform_node
PLUGINLIB_EXPORT_CLASS(turret_transform_node::TurretTransformNode, nodelet::Nodelet)
