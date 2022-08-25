#include <ros/ros.h>
#include<common_settings.hpp>

#include <iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

using namespace boost::property_tree;

namespace parameter_server
{
    class Server{
        public:
           ros::NodeHandle nodehandle_;
            //set parameter to parameter server
            ros::Subscriber sub_;
            //notify that parameter is set or changed
            ros::Publisher pub_;
            //get parameter from parameter server
            ros::ServiceServer service_server_;
            std::map<std::string, double> parameters_;
        public:
            void callback(const common_settings::topic::SetParameter::Message::ConstPtr & msg);
            bool service_callback(common_settings::topic::GetParameter::Message::Request & req, common_settings::topic::GetParameter::Message::Response & res);

        public:
            void start(){
                ros::spin();
            }
            Server(){
                
                nodehandle_;
                sub_= nodehandle_.subscribe<common_settings::topic::SetParameter::Message>(common_settings::topic::SetParameter::name, 1, &Server::callback, this);
                service_server_ = nodehandle_.advertiseService(common_settings::topic::SetParameter::name, &Server::service_callback, this);
                pub_ = nodehandle_.advertise<common_settings::topic::ParameterChangeNotify::Message>(common_settings::topic::ParameterChangeNotify::name, 1);

                //inport setting json
                ROS_WARN("Server is reading json");
                ptree ptree_;
                read_json("settings.json",ptree_);
                // Data.info
                std::string buffer;
                BOOST_FOREACH (const ptree::value_type& child, ptree_.get_child("info")) {
                    const ptree& info = child.second;

                    // Data.info.id
                    if (boost::optional<std::string> id = info.get_optional<std::string>("parameter_name")) {
                        buffer = id.get();
                    }

                    // Data.info.name
                    if (boost::optional<double> name = info.get_optional<double>("parameter")) {
                        parameters_[buffer] = name.get();
                    }
                }
                //inport setting json END


                ROS_WARN("Server is started");
            }

    };

    void Server::callback(const common_settings::topic::SetParameter::Message::ConstPtr & msg){
        //check vailidity of message
        if(msg->parameter_name == ""){
            ROS_WARN("Server: parameter_name is empty");
            return;
        }
        //set parameter and notify
        if(parameters_.find(msg->parameter_name) !=parameters_.end() || parameters_[msg->parameter_name] == msg->parameter ){
            parameters_[msg->parameter_name] = msg->parameter;
            common_settings::topic::ParameterChangeNotify::Message notify_msg;
            notify_msg.parameter_name = msg->parameter_name;
            notify_msg.parameter = msg->parameter;
            pub_.publish(notify_msg);
        }else{
            ROS_WARN("Server: parameter has already been set or it is the same as the current value");
        }
    }

    bool Server::service_callback(common_settings::topic::GetParameter::Message::Request & req, common_settings::topic::GetParameter::Message::Response & res){
        if(parameters_.find(req.parameter_name) != parameters_.end()){
            res.parameter = parameters_[req.parameter_name];
            return true;
        }else{
            return false;
        }
    }

    


    int main(int argc, char **argv){
        ros::init(argc, argv, "parameter_server");
        Server server;

        ros::spin();

        return 0;
    }
} // namespace parameter_server
