
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
            uint16_t id;
            int angle = 0;
            float sensitivity = 1.0;
        public:
            void onInit(){
                nodehandle_ = getNodeHandle();
                can_tx_pub_ = nodehandle_.advertise<common_settings::topic::CanTx::Message>(common_settings::topic::CanTx::name, 1);
                sub_ = nodehandle_.subscribe<common_settings::topic::ElevationAngle::Message>(common_settings::topic::ElevationAngle::name, 1, &TurretTransformNode::callback, this);

                //TODO: get id from parameter server, OR set right id. IT IS TEST PARAMETER.
                id = 0x200;

                NODELET_INFO("TurretWheelTransformNode is initialized");
                NODELET_WARN("TurretWheelTransformNode : id is not implemented yet");
            }

            protected:
            void callback(const common_settings::topic::ElevationAngle::Message::ConstPtr &data){
                //it uses degrees. if shirasu uses radians, it should be converted to radians.
                NODELET_WARN("turrelwheel_transform_node: Use degree method.");
                angle += data->data * sensitivity;
                can_tx_pub_.publish(can_utils::makeFrame(id,angle));
            }
    };

}//namespace turret_wheel_transform_node
PLUGINLIB_EXPORT_CLASS(turret_transform_node::TurretTransformNode, nodelet::Nodelet)
