
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
    class CanOutputNode : public nodelet::Nodelet{
        private:
            ros::Publisher pub_;

        protected:
            ros::Subscriber can_rx_sub_;
            std::function <void(const can_plugins::Frame::ConstPtr&)> callback_;
             ros::NodeHandle& nodehandle_;
        public:
            virtual void onInit(){
                nodehandle_ = getNodeHandle();
                if(topic_name_.empty()){
                    ROS_ASSERT("topic_name_ is not set");
                    ROS_ASSERT("you should set topic_name_ in the derived class");
                    return;
                }
               // can_rx_sub_ = nodehandle_.subscribe("can_rx", 1000,boost::bind(callback_,this, _1));
                pub_ = nodehandle_.advertise<T>(topic_name_, 1000);
            }
        protected:
            std::string topic_name_;
            inline void publish(T msg){
                pub_.publish(msg);
            }

    };
}