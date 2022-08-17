//Author : Yamada

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>

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
            std::string topic_name_;
            //you should override this function
            virtual void can_input(const TConstPtr& data);
        private:
            ros::Subscriber sub_;
            ros::Publisher can_rx_pub_;
            void callback(const TConstPtr& data){
                can_input(data);
            };
        public:
            CanInputNode(){};
            virtual void onInit(){
                ros::NodeHandle& nh_ = getNodeHandle();
                ros::NodeHandle& pnh_ = getPrivateNodeHandle();
                if(topic_name_.empty()){
                    ROS_ASSERT("topic_name_ is not set");
                    ROS_ASSERT("you should set topic_name_ in the derived class");
                    return;
                }
                sub_ = nh_.subscribe<T>(topic_name_, 1000, &CanInputNode::callback);
                can_rx_pub_ = nh_.advertise<can_plugins::Frame>("can_rx", 1000);
            }
        protected:
            inline void publish(can_plugins::Frame frame){
                can_rx_pub_.publish(frame);
            }

    };
}// namespace can_input_node
