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
    //Base class for can_input nodelet
    //This class is used to can_input data from one topic to can_rx topic
    //The template parameter is the type of the data to be can_inputed

    //TODO: you should override the callback function to transform the data and publish it to can_rx topic
    //TODO: you should set topic_name_ in the derived class
    template<typename T, typename TConstPtr>
    class CanInputNode : public nodelet::Nodelet{
        protected: 
            //you should set this in the derived class
            std::string topic_name_;
            //you should set the callback function in the derived class
            //std::function <void(const TConstPtr&)> callback_;
            void (*callback_)(const TConstPtr&);
            ros::NodeHandle& nodehandle_;
            ros::Subscriber sub_;
        private:

            ros::Publisher can_tx_pub_;
        public:
            virtual void onInit(){
                if(topic_name_.empty()||callback_==nullptr){
                    NODELET_WARN("topic_name_ or callback_ are not set");
                    NODELET_WARN("you should set topic_name_ and callback_ in the derived class");
                    return;
                }
                nodehandle_ = getNodeHandle();
                sub_ = nodehandle_.subscribe<T>(topic_name_, 1000, callback_, this);
                can_tx_pub_ = nodehandle_.advertise<can_plugins::Frame>("can_tx", 1000);
            }
        protected:
            inline void publish(can_plugins::Frame frame){
                can_tx_pub_.publish(frame);
            }

    };
}// namespace can_input_node
