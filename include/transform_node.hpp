//Author : Yamada

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
    //This class is used to transform data from one topic to can_rx topic
    //The template parameter is the type of the data to be transformed

    //TODO: you should override the transform function to transform the data
    //TODO: you should set topic_name_ in the derived class
    template<typename T>
    class TransformNode : public nodelet::Nodelet
    {

        protected:
        //set the value of topic_name_ in the derived class
        string topic_name_;
        //override this function to transform the data
        can_plugins::Frame transform(const T data);

        public:
        virtual void onInit(){
            nh_ = getNodeHandle();
            pnh_ = getPrivateNodeHandle();
            can_tx_pub_ = nh_.advertise<can_plugins::Frame>("can_tx", 1000);
            input_ = nh_.subscribe<T>(topic_name_, 1000, &publish, this);
            NODELET_INFO("can_handler has started.");
        };

        private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher can_tx_pub_;
        ros::Subscriber input_;


        void publish(const T data){
            can_plugins::Frame msg;
            msg = transform(data);
            can_tx_pub_.publish(msg);
        };

    };
}// namespace transform_node
