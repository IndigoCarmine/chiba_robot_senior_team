#define TOPIC_NAME "undercarrige_params"


#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <can_plugins/Frame.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>



namespace undercarrige_transform_node{
    class UndercarrigeTransformNode : public nodelet::Nodelet{
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher can_tx_pub_;
        public:
            void onInit(){
                sub_ = nodehandle_.subscribe(TOPIC_NAME, 1000, &UndercarrigeTransformNode::callback, this);
                can_tx_pub_ = nodehandle_.advertise<can_plugins::Frame>("can_tx", 1000);
                NODELET_INFO("UndercarrigeTransformNode is started.");
            }
        private:
            void callback(const geometry_msgs::Vector3::ConstPtr &data){
                //TODO 
                NODELET_WARN("undercarrige_transform_node::undercarrigeCallback is not implemented yet");
               
                can_plugins::Frame frame;
                can_tx_pub_.publish(frame);
                
            }
    };
    
} // namespace undercarrige_transform_node
PLUGINLIB_EXPORT_CLASS(undercarrige_transform_node::UndercarrigeTransformNode, nodelet::Nodelet);