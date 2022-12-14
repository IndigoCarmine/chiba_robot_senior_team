#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <chiba_robot_senior_team/Saver.h>
#include <common_settings.hpp>
using namespace common_settings;




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

            ros::ServiceClient saver_client_;
        public:
            void callback(const topic::SetParameter::Message::ConstPtr & msg);
            bool service_callback(topic::GetParameter::Message::Request & req, topic::GetParameter::Message::Response & res);

            inline void loadParameters();
            inline void dumpParameters();

        public:
            void onInit()override{
                nodehandle_ = getNodeHandle();

                sub_= nodehandle_.subscribe<topic::SetParameter::Message>(topic::SetParameter::name, 1, &ParameterServerNode::callback, this);
                service_server_ = nodehandle_.advertiseService(topic::SetParameter::name, &ParameterServerNode::service_callback, this);
                pub_ = nodehandle_.advertise<topic::ParameterChangeNotify::Message>(topic::ParameterChangeNotify::name, 1);
                saver_client_ = nodehandle_.serviceClient<chiba_robot_senior_team::Saver>("parameter",1);// It is for parameter saver

                loadParameters();


                NODELET_INFO("ParameterServerNode is started.");

                NODELET_WARN("ParameterServerNode: not implemented yet. IT CANNOT SAVE PARAMETERS TO FILE.");
            }

    };

    void ParameterServerNode::callback(const topic::SetParameter::Message::ConstPtr & msg){
        //check vailidity of message
        if(msg->parameter_name == ""){
            NODELET_WARN("ParameterServerNode: parameter_name is empty");
            return;
        }
        //set parameter and notify
        if(parameters_.find(msg->parameter_name) !=parameters_.end() || parameters_[msg->parameter_name] == msg->parameter ){
            //set parameter
            parameters_[msg->parameter_name] = msg->parameter;

            topic::ParameterChangeNotify::Message notify_msg;
            notify_msg.parameter_name = msg->parameter_name;
            notify_msg.parameter = msg->parameter;
            pub_.publish(notify_msg);
        }else{
            NODELET_INFO("ParameterServerNode: parameter has already been set or it is the same as the current value");
        }
    }

    bool ParameterServerNode::service_callback(topic::GetParameter::Message::Request & req, topic::GetParameter::Message::Response & res){
        if(parameters_.find(req.parameter_name) != parameters_.end()){
            res.parameter = parameters_[req.parameter_name];
            return true;
        }else{
            parameters_[req.parameter_name] = 0;
            NODELET_WARN("ParameterServerNode: invalid request coming");
            return false;
        }
    }

    void ParameterServerNode::loadParameters(){
        //load parameters from file
        chiba_robot_senior_team::Saver srv;
        srv.request.save_or_load = false;
        this->saver_client_.call(srv);//wait until finish updating
        //get parameters from rosparam
        if(nodehandle_.hasParam("settings")){
            nodehandle_.getParam("settings",parameters_);
        }else{
            NODELET_WARN("ParameterServerNode: settings is not exist.");
        }
    }

    void ParameterServerNode::dumpParameters(){
        //save parameters to rosparam
        nodehandle_.setParam("settings",parameters_);

        //save parameters to file
        chiba_robot_senior_team::Saver srv;
        srv.request.save_or_load = true;
        this->saver_client_.call(srv);//wait until finish updating
    }


}
PLUGINLIB_EXPORT_CLASS(parameter_server_node::ParameterServerNode, nodelet::Nodelet)