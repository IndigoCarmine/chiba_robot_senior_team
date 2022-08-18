#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>

namespace controller_node{
    class ControllerNode : public nodelet::Nodelet{
        private:
        ros::Subscriber joy_sub_;
        ros::Publisher twist_pub_;
        ros::Publisher actuators_pub_;
        public:
        void onInit(){
            ros::NodeHandle& nh_ = getNodeHandle();
            ros::NodeHandle& pnh_ = getPrivateNodeHandle();
            joy_sub_ = nh_.subscribe("joy", 1000, &ControllerNode::joyCallback, this);
            twist_pub_ = nh_.advertise<geometry_msgs::Vector3>("undercarrige_params", 10);
            actuators_pub_ = nh_.advertise<std_msgs::Int32>("", 1000);
            NODELET_INFO("controller_node has started.");
        }
        private:

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){

            //publish twist message
            geometry_msgs::Vector3 vecter;
            vecter.x = msg->axes[0];
            vecter.y = msg->axes[1];
            vecter.z = msg->axes[2];
            twist_pub_.publish(vecter);
            //publish button message
            //TODO
            NODELET_WARN("I don't know what to do with this message. Sorry.");
            NODELET_WARN("You should write joyCallback in ControllerNode.");

            //actuators_pub_.publish(something);

        }
    };
} // namespace controller_node
PLUGINLIB_EXPORT_CLASS(controller_node::ControllerNode, nodelet::Nodelet);