#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "common_settings.hpp"
using namespace common_settings;
#include<queue>

using namespace common_settings;
namespace work_conductor_node{
    enum WorkState{
        MOVE_RIGHT,
        COLLECT,
        SHOT,
        FINISH,
        STOP,
        ERROR
    };
    enum WorkerID{
        MOVE_WORK_NODE,
        SHOT_WORK_NODE,
        CONDUCT_WORK_NODE,
        DEPRIVE_WORK_AUTHORITY
    };
    //The class is conductor node for automatic work
    class WorkConductorNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber joy_sub_;
            ros::Publisher work_pub_;//publish work state
            ros::Subscriber work_sub_;//subscribe work state
            ros::Timer timer_;
            int work_state_;//work state
            //work state: 0:move right, 1:collect, 2:shot, 3:finish 4:stop 5:error
            bool is_auto_ = false;//is automatic work

        public:
        void onInit(){
            nh_ = getNodeHandle();
            joy_sub_ = nh_.subscribe(topic::JoystickParams::name, 1, &WorkConductorNode::joyCallback, this);
            work_pub_ = nh_.advertise<topic::Work::Message>(topic::Work::name, 1);
            work_sub_ = nh_.subscribe<topic::Work::Message>(topic::Work::name, 1, &WorkConductorNode::workCallback, this);
            timer_ = nh_.createTimer(ros::Duration(0.1), &WorkConductorNode::timerCallback, this);
            NODELET_INFO("work_conductor_node has started.");
        }

        private:
        void joyCallback(const  topic::JoystickParams::Message::ConstPtr& msg){

            if(msg->header.frame_id== "auto"){
                is_auto_ = true;
            }
            else{
                is_auto_ = false;
                topic::Work::Message work_msg;
                work_msg.worker_id =3;//deprive work authority
                work_pub_.publish(work_msg);
            }
        }
        void workCallback(const topic::Work::Message::ConstPtr& msg){
            //TODO
            NODELET_WARN("You should write workCallback in WorkConductorNode.");
        }
        void timerCallback(const ros::TimerEvent& event){
            //TODO
            NODELET_WARN("You should write timerCallback in WorkConductorNode.");
            //Worker ID: 
            //  0: MoveWorkNode
            //  1: ShotWorkNode
            //  2: ConductWorkNode
            //  3: deprive work authority or emargency stop
            if(is_auto_){
                topic::Work::Message work_msg;
                switch (work_state_)
                {
                case WorkState::MOVE_RIGHT:
                    //active ModeWorkerNode 
                    work_msg.worker_id = 0;
                    break;
                case WorkState::COLLECT:
                    //active ModeWorkerNode 
                    work_msg.worker_id = 1;
                    break;
                case WorkState::SHOT:
                    //active ModeWorkerNode 
                    work_msg.worker_id = 2;
                    break;
                default:
                    work_msg.worker_id = 3;//deprive work authority
                    break;
                }
                work_pub_.publish(work_msg);
            }

        }
    };


}//namespace work_conductor_node
PLUGINLIB_EXPORT_CLASS(work_conductor_node::WorkConductorNode, nodelet::Nodelet)




//Worker ID: 
//  0: MoveWorkNode
//  1: ShotWorkNode
//  2: ConductWorkNode
//  3: deprive work authority or emargency stop
