#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Int16.h>
#include <pluginlib/class_list_macros.hpp>
#include "common_settings.hpp"
using namespace common_settings;
namespace shot_work_node
{
    class ShotWorkNode : public nodelet::Nodelet
    {
        private:
            ros::NodeHandle nh_;
            ros::Subscriber work_sub_;

        public:
        void onInit()
        {
            nh_ = getNodeHandle();
            work_sub_ = nh_.subscribe<topic::Work::Message>(topic::Work::name, 1, &ShotWorkNode::workCallback, this);
            NODELET_INFO("shot_work_node has started.");
        }

        private:
        void workCallback(const topic::Work::Message::ConstPtr &state)
        {

            //TODO
            NODELET_WARN("You should write workCallback in ShotWorkNode.");
        }
    };
} // namespace shot_work_node
PLUGINLIB_EXPORT_CLASS(shot_work_node::ShotWorkNode, nodelet::Nodelet)