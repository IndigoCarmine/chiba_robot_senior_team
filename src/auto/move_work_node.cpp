#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Int16.h>
#include "common_settings.hpp"
using namespace common_settings;
#include <array>
namespace move_work_node
{
    
    //This class is for moving to upper right.
    //It control undercarriage so that it moves to upper right.
    //If the sensor detects the obstacle (fence in front of the unit), it move only right.
    //If the sensor detects the obstacle (fence in right of the unit), it move only upper.
    //If the sensor detects the obstacle (fence in front of the unit and right of the unit), it stop and publish the message to the topic "work".
    class MoveWorkNode: public nodelet::Nodelet
    {
        private:
            ros::NodeHandle nh_;
            ros::Subscriber work_sub_;
            ros::Publisher undercarriage_pub_;
            ros::Subscriber unit_position_sub_;
            ros::Subscriber sensor_sub_;
            ros::Timer timer_;

            std::array<double, 2> unit_position_ = {0.0, 0.0};
            double unit_angle_ = 0.0;
            ros::Time positon_update_time_;

            float front_right_obstacle_distance_ = INFINITY;
            ros::Time front_right_obstacle_update_time_;
            float front_left_obstacle_distance_ = INFINITY;
            ros::Time front_left_obstacle_update_time_;

            float right_obstacle_distance_  = INFINITY;
            ros::Time right_obstacle_update_time_;
            

            //work parameter
            float error_duration_ = 0.5;//[s] if the sensor does not update for this duration, it is error
            float max_speed_ = 0.5;//[m/s]
            float max_angular_speed_ = 0.5;//[rad/s]

            float rotation_activation_scale_ = 0.5;
            
            float front_obstacle_distance_threshold_ = 0.5;//[m] demand distance from the obstacle in front of the unit
            float right_obstacle_distance_threshold_ = 0.5;//[m] demand distance from the obstacle in right of the unit
            float sensor_activation_scale_ = 0.5;//[/m] activation scale of the sensor
        public:
        void onInit()
        {
            nh_ = getNodeHandle();
            work_sub_ = nh_.subscribe<topic::Work::Message>(topic::Work::name, 1, &MoveWorkNode::workCallback, this);
            undercarriage_pub_ = nh_.advertise<common_settings::topic::UndercarriageParams::Message>(common_settings::topic::UndercarriageParams::name, 1);
            unit_position_sub_ = nh_.subscribe<common_settings::topic::UnitPositionParams::Message>(common_settings::topic::UnitPositionParams::name, 1, &MoveWorkNode::unitPositionCallback, this);
            sensor_sub_ = nh_.subscribe<common_settings::topic::SensorParams::Message>(common_settings::topic::SensorParams::name, 1, &MoveWorkNode::sensorCallback, this);
            timer_ = nh_.createTimer(ros::Duration(0.1), &MoveWorkNode::timerCallback, this,false);
            NODELET_INFO("move_work_node has started.");

            //init time 
            positon_update_time_ = ros::Time::now();
            front_right_obstacle_update_time_ = ros::Time::now();
            front_left_obstacle_update_time_ = ros::Time::now();
            right_obstacle_update_time_ = ros::Time::now();
        }

        private:
        void workCallback(const topic::Work::Message::ConstPtr &state)
        {    
            if(state->worker_id == 1){
                timer_.start();
            }else{
                timer_.stop();
            }
        }


        void timerCallback(const ros::TimerEvent& event)
        {
            //show error if the position is not updated for a while
            if(ros::Time::now() - positon_update_time_ > ros::Duration(error_duration_))NODELET_ERROR("unit position is not updated.");
            if(ros::Time::now() - front_right_obstacle_update_time_ > ros::Duration(error_duration_))NODELET_ERROR("front right obstacle is not updated.");
            if(ros::Time::now() - front_left_obstacle_update_time_ > ros::Duration(error_duration_))NODELET_ERROR("front left obstacle is not updated.");
            if(ros::Time::now() - right_obstacle_update_time_ > ros::Duration(error_duration_))NODELET_ERROR("right obstacle is not updated.");

            topic::UndercarriageParams::Message undercarriage_msg;//It is velocity of the undercarriage.
            // revise rotaiton: if it does not face to the upper, it rotates to the upper.
            undercarriage_msg.angular.z = activationFunction(unit_angle_,rotation_activation_scale_)* (ros::Time::now() - event.last_real).toSec();
            //It control undercarriage so that it moves to upper right.
            //If the sensor detects the obstacle (fence in front of the unit), it move only right.
            //If the sensor detects the obstacle (fence in right of the unit), it move only upper.
            //If the sensor detects the obstacle (fence in front of the unit and right of the unit), it stop and publish the message to the topic "work".
            //upper is axis y and right is axis x.
            if(front_right_obstacle_distance_ < front_obstacle_distance_threshold_ && right_obstacle_distance_ < right_obstacle_distance_threshold_){
                undercarriage_msg.linear.x = 0.0;
                undercarriage_msg.linear.y = 0.0;
            }else if(front_right_obstacle_distance_ < front_obstacle_distance_threshold_){
                undercarriage_msg.linear.x = 0.0;
                undercarriage_msg.linear.y = max_speed_ * activationFunction(right_obstacle_distance_ - right_obstacle_distance_threshold_,sensor_activation_scale_);
            }else if(right_obstacle_distance_ < right_obstacle_distance_threshold_){
                undercarriage_msg.linear.x = max_speed_ * activationFunction(front_right_obstacle_distance_ - front_obstacle_distance_threshold_,sensor_activation_scale_);
                undercarriage_msg.linear.y = 0.0;
            }else{
                undercarriage_msg.linear.x = max_speed_ * activationFunction(front_right_obstacle_distance_ - front_obstacle_distance_threshold_,sensor_activation_scale_);
                undercarriage_msg.linear.y = max_speed_ * activationFunction(right_obstacle_distance_ - right_obstacle_distance_threshold_,sensor_activation_scale_);
            }
            undercarriage_pub_.publish(undercarriage_msg);
        }
        
        void unitPositionCallback(const common_settings::topic::UnitPositionParams::Message::ConstPtr& msg)
        {
            //update unit position
            unit_position_[0] = msg->x;
            unit_position_[1] = msg->y;
            unit_angle_ = msg->z;
            positon_update_time_ = ros::Time::now();
        }
        void sensorCallback(const common_settings::topic::SensorParams::Message::ConstPtr& msg)
        {
            //sensorID 0: front right obstacle sensor
            //sensorID 1: front left obstacle sensor
            //sensorID 2: right obstacle sensor
            //update sensor data
            switch (msg->sensor_id)
            {
            case 0:
                front_right_obstacle_distance_ = msg->value;
                front_right_obstacle_update_time_ = ros::Time::now();
                break;
            case 1:
                front_left_obstacle_distance_ = msg->value;
                front_left_obstacle_update_time_ = ros::Time::now();
                break;
            case 2:
                right_obstacle_distance_ = msg->value;
                right_obstacle_update_time_ = ros::Time::now();
                break;
            default:
                NODELET_WARN("MoveWorkNode:sensor id is invalid.");
                break;
            }
        }
        
        inline double activationFunction(double x,double scale = 1.0)
        {
            return 1.0/(1.0 + exp(-x*scale));
        }
    };


} // namespace move_work_node
PLUGINLIB_EXPORT_CLASS(move_work_node::MoveWorkNode, nodelet::Nodelet)