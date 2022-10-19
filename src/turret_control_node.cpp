#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Vector3.h>
#include <can_utils_rev.hpp>
#include <can_plugins/Frame.h>
#include <common_settings.hpp>

namespace turret_control_node{
    
    class TurretControlNode: public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber joystick_sub_;
            ros::Publisher turret_pub;

            //it is for manual control
            double evaluation_angle_ = 0;
            double speed_ = 0;

        public:
            void onInit(){
                nodehandle_ = getNodeHandle();
                joystick_sub_ = nodehandle_.subscribe(common_settings::topic::JoystickParams::name, 1, &TurretControlNode::callback, this);
                turret_pub = nodehandle_.advertise<common_settings::topic::TurretParams::Message>(common_settings::topic::TurretParams::name, 1);

                //TODO: get id from parameter server, OR set right id. IT IS TEST PARAMETER.

                NODELET_INFO("TurretControlNode is initialized");
                NODELET_WARN("TurretControlNode : id is not implemented yet");
                NODELET_WARN("TurretControlNode :  not implemented yet");
            }

            protected:
            void callback(const common_settings::topic::JoystickParams::Message::ConstPtr &data){
                //it uses degrees. if shirasu uses radians, it should be converted to radians.
                NODELET_WARN("turrelwheel_control_node: Use degree method.");
                //if it is not auto mode, all speed is same.
                if(data->header.frame_id == common_settings::frame_id::auto_mode){    

                }else{
                    //manual mode
                    //integrate evaluation angle

                    //make Trret Message and publish it.
                    
                    common_settings::topic::TurretParams::Message msg;
                    msg.evaluation_angle = 
                }

                
            }
    };

}//namespace turret_transform_node
PLUGINLIB_EXPORT_CLASS(turret_control_node::TurretControlNode, nodelet::Nodelet)
