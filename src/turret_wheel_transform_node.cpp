
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <can_plugins/Frame.h>
#include <geometry_msgs/Vector3.h>
#include <can_utils_rev.hpp>
#include <can_plugins/Frame.h>
#include <common_settings.hpp>

namespace turret_wheel_transform_node{
    
    class TurretWheelTransformNode: public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher can_tx_pub_;
            uint16_t id;
        public:
            void onInit(){
                nodehandle_ = getMTNodeHandle();
                can_tx_pub_ = nodehandle_.advertise<can_plugins::Frame>(common_settings::can_tx, 1000);
                sub_ = nodehandle_.subscribe(common_settings::turret_wheel_params, 1, &TurretWheelTransformNode::callback, this);

                //TODO: get id from parameter server, OR set right id. IT IS TEST PARAMETER.
                id = 0x200;

                NODELET_INFO("TurretWheelTransformNode is initialized");
                NODELET_WARN("TurretWheelTransformNode : id is not implemented yet");
            }
            protected:
            void callback(const geometry_msgs::Vector3::ConstPtr &data){
                //it uses degrees. if shirasu uses radians, it should be converted to radians.
                NODELET_WARN("turrelwheel_transform_node: Use degre method.");
                //transform vector3 to degrees. and y axis is 0.
                float degree = std::atan(data->y/data->x)*180/M_PI +90;
                //transform degree to can fram and publish it.
                can_tx_pub_.publish(can_utils::makeFrame(id, degree));
                
            }
    };

}//namespace turret_wheel_transform_node
PLUGINLIB_EXPORT_CLASS(turret_wheel_transform_node::TurretWheelTransformNode, nodelet::Nodelet);