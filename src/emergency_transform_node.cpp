#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>
#include <can_utils_rev.hpp>
#include <can_plugins/Frame.h>
#include "common_settings.hpp"
using namespace common_settings;

#include <vector>  

namespace emergency_transform_node{
    class EmergencyTransformNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Publisher can_tx_pub_;
            ros::Subscriber sub_;
            std::vector<uint16_t> id_list_;
        public:
            void onInit(){
                nodehandle_ = getNodeHandle();
                can_tx_pub_ = nodehandle_.advertise<topic::CanTx::Message>(topic::CanTx::name, 1);
                sub_ = nodehandle_.subscribe<topic::EmergencyCmd::Message>(topic::EmergencyCmd::name, 1, &EmergencyTransformNode::callback, this);
                id_list_ = {0x00,0x01};
                NODELET_INFO("EmergencyTransformNode is started.");  
                NODELET_WARN("EmergencyTransformNode : id_list_ is not implemented yet");
            }
        private:
            void callback(const std_msgs::Bool::ConstPtr &data){
                NODELET_WARN("EmergencyTransformNode : id_list_ is not implemented yet");
                
                if(data->data){
                    //all moters are stopped
                    for(int i =0; i<50;i++){
                        for(uint16_t id : id_list_){
                        can_tx_pub_.publish(can_utils::makeFrame(id, can_utils::Command::shutdown));
                        }
                    }

                }else{
                    //all moters are restarted
                    for(uint16_t id : id_list_){
                        can_tx_pub_.publish(can_utils::makeFrame(id, can_utils::Command::recover));
                        NODELET_WARN("if Shirasu formware don't set default mode, it happens that the motor is not working or undefined behavior.");
                        //if Shirasu formware don't set default mode, it happens that the motor is not working or undefined behavior.
                        //you should use the following code. but it forces the motor to be in velocity mode.
                        //can_tx_pub_.publish(can_utils::makeFrame(id, can_utils::Command::recover_velocity));
                    }
                }


            }
    };
}//namespace emergency_transform_node
PLUGINLIB_EXPORT_CLASS(emergency_transform_node::EmergencyTransformNode, nodelet::Nodelet)