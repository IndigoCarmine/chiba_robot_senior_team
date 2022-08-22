#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <common_settings.hpp>



namespace parameter_server_node{
    //it is like dynamic reconfigure on nodelet
    //it can be used to change double parameter on runtime
    class SetParameterNode : public nodelet::Nodelet
    {
        private:
            ros::NodeHandle nodehandle_;
            ros::Subscriber sub_;
            ros::Publisher pub_;
            ros::ServiceServer service_server_;
            std::map<std::string, double> parameters_;
        public:
            void callback(const common_settings::topic::SetParameter::Message::ConstPtr & msg);
            bool service_callback(common_settings::topic::GetParameter::Message::Request & req, common_settings::topic::GetParameter::Message::Response & res);

        public:
            void onInit()override{
                nodehandle_ = getNodeHandle();
                //set parameter to parameter server
                sub_= nodehandle_.subscribe<common_settings::topic::SetParameter::Message>(common_settings::topic::SetParameter::name, 1, &SetParameterNode::callback, this);
                //get parameter from parameter server
                service_server_ = nodehandle_.advertiseService(common_settings::topic::SetParameter::name, &SetParameterNode::service_callback, this);

                //notify that parameter is set or changed
                pub_ = nodehandle_.advertise<common_settings::topic::ParameterChangeNotify::Message>(common_settings::topic::ParameterChangeNotify::name, 1);

                NODELET_INFO("SetParameterNode is started");
            }
    };

    void SetParameterNode::callback(const common_settings::topic::SetParameter::Message::ConstPtr & msg){
        //check vailidity of message
        if(msg->parameter_name == ""){
            NODELET_WARN("SetParameterNode: parameter_name is empty");
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
            NODELET_INFO("SetParameterNode: parameter has already been set or it is the same as the current value");
        }
    }

    bool SetParameterNode::service_callback(common_settings::topic::GetParameter::Message::Request & req, common_settings::topic::GetParameter::Message::Response & res){
        if(parameters_.find(req.parameter_name) != parameters_.end()){
            res.parameter = parameters_[req.parameter_name];
            return true;
        }else{
            return false;
        }
    }


}