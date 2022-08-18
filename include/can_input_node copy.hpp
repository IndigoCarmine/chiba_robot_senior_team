//Author : Yamada

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <can_plugins/Frame.h>

#include <stdint.h>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace can_input_node{
    class CanInputNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle& nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher can_tx_pub_;
        public:
            virtual void onInit(){
                nodehandle_ = getNodeHandle();
                sub_ = nodehandle_.subscribe<T>(topic_name_, 1000, callback_, this);
                can_tx_pub_ = nodehandle_.advertise<can_plugins::Frame>("can_tx", 1000);
            }
        private:
            inline void publish(can_plugins::Frame frame){
                can_tx_pub_.publish(frame);
            }

    };
}// namespace can_input_node
