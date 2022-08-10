#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>

#include <can_plugins/Frame.h>
#include "can_utils.hpp"

#include <stdint.h>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace transform_node{
    //Base class for transform nodelet
    //This class is used to transform data from one topic to can_tx topic
    //The template parameter is the type of the data to be transformed
    //you should override the transform function to transform the data
    template<typename T>
    class TransformNode : public nodelet::Nodelet
    {
        public:
        virtual void onInit(){
            _nh = getNodeHandle();
            pnh = getPrivateNodeHandle();
            _can_tx_pub = _nh.advertise<can_plugins::Frame>("can_tx", 1000);
            _input = _nh.subscribe<T>(_topic, 1000, &publish, this);
            NODELET_INFO("can_handler has started.");
        };

        private:
        ros::NodeHandle _nh;
        ros::NodeHandle pnh;
        ros::Publisher _can_tx_pub;
        ros::Subscriber _input;
        void publish(const T data){
            can_plugins::Frame msg;
            msg = transform(data);
            _can_tx_pub.publish(msg);
        };

        protected:
        string _topic;
        virtual can_plugins::Frame transform(const T data);
    };
}// namespace transform_node
