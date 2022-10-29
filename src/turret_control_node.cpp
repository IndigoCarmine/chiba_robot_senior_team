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
                NODELET_WARN("TurretControlNode :  We should set limitation of evaluation angle"); 
            }

            protected:
            void callback(const common_settings::topic::JoystickParams::Message::ConstPtr &data){
                //it uses degrees. if shirasu uses radians, it should be converted to radians.
                NODELET_WARN("turrelwheel_control_node: Use degree method.");
                //if it is not auto mode, all speed is same.
                if(data->header.frame_id == common_settings::frame_id::auto_mode){    

                }else if(data->header.frame_id == common_settings::frame_id::aiming_mode){
                    //aiming mode
                    //integrate evaluation angle
                    evaluation_angle_ += data->twist.linear.y;
                    //limit evaluation angle
                    if(evaluation_angle_ > 90){
                        evaluation_angle_ = 90;
                    }else if(evaluation_angle_ < 0){
                        evaluation_angle_ = 0;
                    }
                    //set speed
                    speed_ += data->twist.linear.x;
                    //limit speed in 0 to 243
                    if(speed_ > 244){
                        speed_ = 244;
                    }else if(speed_ < 0){
                        speed_ = 0;
                    }
                    //make Trret Message and publish it.
                    common_settings::topic::TurretParams::Message msg;
                    msg.evaluation_angle  = evaluation_angle_;
                    msg.speed_upper = speed_;
                    msg.speed_lower = speed_;
                    msg.speed_left  = speed_;
                    msg.speed_right = speed_;
                    turret_pub.publish(msg);
                }
            }


    };

}//namespace turret_transform_node
PLUGINLIB_EXPORT_CLASS(turret_control_node::TurretControlNode, nodelet::Nodelet)
