#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace controller_node{
    class ControllerNode : public nodelet::Nodelet{
        private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber joy_sub_;
        ros::Publisher twist_pub_;
        public:
        virtual void onInit(){
            nh_ = getNodeHandle();
            pnh_ = getPrivateNodeHandle();
            joy_sub_ = nh_.subscribe("joy", 1000, &ControllerNode::joyCallback, this);
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
            ROS_INFO("controller_node has started.");
        };
        private:

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
            geometry_msgs::Twist twist;
            twist.linear.x = msg->axes[1];
            twist.linear.y = msg->axes[0];
            twist.linear.z = msg->axes[3];
            twist.angular.x = msg->axes[4];
            twist.angular.y = msg->axes[5];
            twist.angular.z = msg->axes[2];
            twist_pub_.publish(twist);
        };
    };
} // namespace controller_node

PLUGINLIB_EXPORT_CLASS(controller_node::ControllerNode, nodelet::Nodelet);