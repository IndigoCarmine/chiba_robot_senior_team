
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <can_plugins/Frame.h>
#include <ros/ros.h>

namespace can_output_node{
    //Base class for can_input nodelet
    //You use the class to change the canframe data and publish it to an output topic 
    //The template parameter is the type of the data

    //TODO: you should override can_output function to transform the data and publish it to an output topic
    //TODO: you should set topic_name_ in the derived class
    template<typename T>
    class CanOutputNode : nodelet::Nodelet{
        public:
            virtual void onInit(){
                ros::NodeHandle& nh = getNodeHandle();
                ros::NodeHandle& private_nh = getPrivateNodeHandle();
                if(topic_name_.empty()){
                    ROS_ASSERT("topic_name_ is not set");
                    ROS_ASSERT("you should set topic_name_ in the derived class");
                    return;
                }
                can_rx_sub_ = nh.subscribe("can_rx", 1000, &CanOutputNode::can_output, this);
                pub_ = nh.advertise<T>(topic_name_, 1000);

            };
        protected:
            std::string topic_name_;
            virtual void can_output(can_plugins::Frame data);    
            inline void publish(T msg){
                pub_.publish(msg);
            };
        private:
            ros::Subscriber can_rx_sub_;
            ros::Publisher pub_;
    };
}