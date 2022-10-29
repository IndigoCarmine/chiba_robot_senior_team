#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Int16.h>
#include <pluginlib/class_list_macros.hpp>

namespace collect_work_node
{
    class CollectWorkNode: public nodelet::Nodelet
    {
        private:
            ros::NodeHandle nh_;
            ros::Subscriber work_sub_;

        public:
        void onInit()
        {
            nh_ = getNodeHandle();
            work_sub_ = nh_.subscribe<std_msgs::Int16>("work", 1, &CollectWorkNode::workCallback, this);
            NODELET_INFO("collect_work_node has started.");
        }

        private:
        void workCallback(const std_msgs::Int16::ConstPtr &state)
        {

            //TODO
            NODELET_WARN("You should write workCallback in CollectWorkNode.");
        }


    };

} // namespace collect_work_node
PLUGINLIB_EXPORT_CLASS(collect_work_node::CollectWorkNode, nodelet::Nodelet)