#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include "common_settings.hpp"
#include "can_utils_rev.hpp"
#include <array>

namespace sensor_transform_node
{
    struct Sensor
    {
        uint32_t can_id;
        uint8_t sensor_id;
    };
    

    class SensorTransformNode:public nodelet::Nodelet{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber can_sub_;
            ros::Publisher sensor_pub_;
            std::array<Sensor,3> sensors_ = {Sensor{0x100,0},Sensor{0x101,1},Sensor{0x102,2}};
        public:
        void onInit(){
            nh_ = getNodeHandle();
            can_sub_ = nh_.subscribe(common_settings::topic::CanRx::name, 1, &SensorTransformNode::canCallback, this);
            sensor_pub_ = nh_.advertise<common_settings::topic::SensorParams::Message>(common_settings::topic::SensorParams::name, 1);
            NODELET_WARN("sensor_transform_node: canID is not set.");
        }
        private:
        void canCallback(const common_settings::topic::CanRx::Message::ConstPtr& msg){
            for(auto sensor:sensors_){
                if(msg->id == sensor.can_id){
                    common_settings::topic::SensorParams::Message sensor_msg;
                    sensor_msg.sensor_id = sensor.sensor_id;
                    sensor_msg.value = can_utils::unpack<float>(msg->data,can_utils::Endian::LITTLE);
                    sensor_pub_.publish(sensor_msg);
                }
            }
        }
    };
    
    
} // namespace sensor_transform_node
PLUGINLIB_EXPORT_CLASS(sensor_transform_node::SensorTransformNode, nodelet::Nodelet)
