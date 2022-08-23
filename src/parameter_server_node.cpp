#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <common_settings.hpp>
using namespace common_settings::topic;

#include <iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
using namespace boost::property_tree;

namespace parameter_server_node{
    //it is like dynamic reconfigure on nodelet
    //it can be used to change double parameter on runtime
    class ParameterServerNode : public nodelet::Nodelet
    {
        private:
            ros::NodeHandle nodehandle_;
            //set parameter to parameter server
            ros::Subscriber sub_;
            //notify that parameter is set or changed
            ros::Publisher pub_;
            //get parameter from parameter server
            ros::ServiceServer service_server_;
            std::map<std::string, double> parameters_;
        public:
            void callback(const SetParameter::Message::ConstPtr & msg);
            bool service_callback(GetParameter::Message::Request & req, GetParameter::Message::Response & res);

        public:
            void onInit()override{
                
                nodehandle_ = getNodeHandle();
                sub_= nodehandle_.subscribe<SetParameter::Message>(SetParameter::name, 1, &ParameterServerNode::callback, this);
                service_server_ = nodehandle_.advertiseService(SetParameter::name, &ParameterServerNode::service_callback, this);
                pub_ = nodehandle_.advertise<ParameterChangeNotify::Message>(ParameterChangeNotify::name, 1);

                //inport setting json
                NODELET_WARN("ParameterServerNode is reading json");
                ptree ptree_;
                read_json("settings.json",ptree_);
                // Data.info
                // std::string buffer;
                // BOOST_FOREACH (const ptree::value_type& child, ptree_.get_child("info")) {
                //     const ptree& info = child.second;

                //     // Data.info.id
                //     if (boost::optional<std::string> id = info.get_optional<std::string>("parameter_name")) {
                //         buffer = id.get();
                //     }

                //     // Data.info.name
                //     if (boost::optional<double> name = info.get_optional<double>("parameter")) {
                //         parameters_[buffer] = name.get();
                //     }
                // }
                //inport setting json END


                NODELET_WARN("ParameterServerNode is started");
            }

    };

    void ParameterServerNode::callback(const SetParameter::Message::ConstPtr & msg){
        //check vailidity of message
        if(msg->parameter_name == ""){
            NODELET_WARN("ParameterServerNode: parameter_name is empty");
            return;
        }
        //set parameter and notify
        if(parameters_.find(msg->parameter_name) !=parameters_.end() || parameters_[msg->parameter_name] == msg->parameter ){
            parameters_[msg->parameter_name] = msg->parameter;
            ParameterChangeNotify::Message notify_msg;
            notify_msg.parameter_name = msg->parameter_name;
            notify_msg.parameter = msg->parameter;
            pub_.publish(notify_msg);
        }else{
            NODELET_INFO("ParameterServerNode: parameter has already been set or it is the same as the current value");
        }
    }

    bool ParameterServerNode::service_callback(GetParameter::Message::Request & req, GetParameter::Message::Response & res){
        if(parameters_.find(req.parameter_name) != parameters_.end()){
            res.parameter = parameters_[req.parameter_name];
            return true;
        }else{
            return false;
        }
    }


}
PLUGINLIB_EXPORT_CLASS(parameter_server_node::ParameterServerNode, nodelet::Nodelet)