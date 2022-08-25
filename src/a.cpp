#include <server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <chiba_robot_senior_team/ParameterConfig.h>
namespace test
{
    class test: nodelet::Nodelet{
        private:
        ros::NodeHandle nodehandle_;
        dynamic_reconfigure::Server server;
        void callback(ros_lecture_msgs::Sample1Config& config, uint32_t level)
        {
        ROS_INFO("Reconfigure Request: %d %f %s %s %d", config.int_param, config.double_param, config.str_param.c_str(),
                config.bool_param ? "True" : "False", config.size);
        }    

        public:
            void onInit()override{
                nodehandle_ = getNodeHandle();
                server.onInit(nodehandle_);
                
            }
    }
} // namespace test
