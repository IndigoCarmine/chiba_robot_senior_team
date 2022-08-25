#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_plugins/Frame.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <array>
#include <numeric>

#include <can_utils_rev.hpp>
#include "common_settings.hpp"


namespace undercarrige_transform_node{

    //The class have specific information about a moter and can calculate speed.
    class Moter{
        public:
            int id;
            int distance;
            std::array<int,2> direction;
            static constexpr double linear_rate = 1;
            static constexpr double angular_rate = 1;
            
        public:
            Moter(const uint16_t id, const int distance, const std::array<int,2> 
                direction):id(id),distance(distance),direction(direction){}

            //Calcuate motor speed
            double calcutateSpeed(const std::array<double,2> &linear, const std::array<double,2> &angular){
                return linear_rate * std::inner_product(linear.begin(), linear.end(), direction.begin(), 0.0) + angular_rate* std::inner_product(angular.begin(), angular.end(), direction.begin(), 0.0);
            }
    };
    

    class UndercarrigeTransformNode : public nodelet::Nodelet{
        private:
            //TODO : set right parameters.
            std::array<Moter,4> moters = {
                Moter(1,5,{0,1}),
                Moter(1,5,{0,1}),
                Moter(1,5,{0,1}),
                Moter(1,5,{0,1}),
            };
            ros::NodeHandle nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher can_tx_pub_;
        public:
            void onInit(){
                nodehandle_ = getNodeHandle();
                sub_ = nodehandle_.subscribe<common_settings::topic::UndercarriageParams::Message>(common_settings::topic::UndercarriageParams::name, 1, &UndercarrigeTransformNode::callback, this);
                can_tx_pub_ = nodehandle_.advertise<common_settings::topic::CanTx::Message>(common_settings::topic::CanTx::name, 1);
                NODELET_INFO("UndercarrigeTransformNode is started.");
            }
        private:
            void callback(const geometry_msgs::Twist::ConstPtr &data){
                NODELET_WARN("UndercarrigeTransformNode : moters is test param.");

                //Calculate the speed of each motor and publish the can frame.
                for(unsigned int i = 0; i < moters.size(); i++){
                    can_plugins::Frame frame = can_utils::makeFrame(moters[i].id, moters[i].calcutateSpeed({data->linear.x, data->linear.y}, {data->angular.x, data->angular.y}));
                    can_tx_pub_.publish(frame);
                }
                
            }
    };

} // namespace undercarrige_transform_node
PLUGINLIB_EXPORT_CLASS(undercarrige_transform_node::UndercarrigeTransformNode, nodelet::Nodelet)