#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/array.hpp>
#include <can_plugins/Frame.h>
#include <can_utils_rev.hpp>
#include "common_settings.hpp"
using namespace common_settings;

#include <stdint.h>
#include <string>
#include <map>

namespace id_manager_node{
  //This node gets CAN ID with getParam.
  //I hope Shirasu returns the ID well, but it probably won't do it.
  //Honestly, I want that Shirasu has a unique name (like MAC Adress) and this node assigns IDs.
  //IDs should be maintained by computers, and humans should maintain easily comprehended and understandable names.
  //I want to make a CONPREHENSIBLE map (e.g.  "right front wheel" --> 0x23)
  class IDManagerNode: public nodelet::Nodelet
  {
    public:
      void onInit()override;

    private:
      void canRxCallback(const topic::CanRx::Message::ConstPtr &msg);
      bool idNameServiceCallback(topic::GetID::Message::Request &msg, topic::GetID::Message::Response &res);
      void bindIDCallback(const topic::BindID::Message::ConstPtr &msg);

      ros::NodeHandle nodehandle_;
      ros::Publisher can_tx_pub_;
      ros::Subscriber can_rx_sub_;
      ros::ServiceServer id_name_service_;
      ros::Subscriber sub_;
      std::map<std::string,std::uint32_t> id_dictionary_;
      
  };

  void IDManagerNode::onInit(){
    nodehandle_ = getNodeHandle();

    can_tx_pub_	= nodehandle_.advertise<topic::CanTx::Message>(topic::CanTx::name, 1);
    can_rx_sub_	= nodehandle_.subscribe<topic::CanRx::Message>(topic::CanRx::name, 1, &IDManagerNode::canRxCallback, this);
    id_name_service_ = nodehandle_.advertiseService("id_name_service",&IDManagerNode::idNameServiceCallback,this);
    sub_ = nodehandle_.subscribe<topic::BindID::Message>(topic::BindID::name,1,&IDManagerNode::bindIDCallback,this);
    NODELET_INFO("IDManagerNode has started.");

    if(nodehandle_.hasParam("id")){
      //get id from parameterserver
      //we cannnot map<std::string,uint32_t>, so firstly use std::map<std::string,int>
      //And transform to map<std::string,uint32_t>
      std::map<std::string,int> id_dictionary;
      nodehandle_.getParam("id",id_dictionary);
      for(auto itr = id_dictionary.begin(); itr != id_dictionary.end(); ++itr) {
          id_dictionary_[ itr->first] = itr->second;
      }

    }else{
        NODELET_WARN("ParameterServerNode: settings is not exist.");
    }  
    //
  }


  void IDManagerNode::canRxCallback(const can_plugins::Frame::ConstPtr &msg){
    //TODO
    NODELET_WARN("I don't know what to do with this message. Sorry.");
    NODELET_WARN("You should write canRxCallback in IDManagerNode.");
    //if the data is a id response, set the id to the dictionary

  }
  enum class ServiceRequestMessage{
    UpdateAllID,
    getID,
  };

  bool IDManagerNode::idNameServiceCallback( topic::GetID::Message::Request &req, topic::GetID::Message::Response &res){
    NODELET_INFO("id_manager_node: id_name_service_callback");
    NODELET_WARN("You should write canRxCallback in IDManagerNode.");
    if(id_dictionary_.find(req.id_name) != id_dictionary_.end()){
      res.id = id_dictionary_[req.id_name];
      return true;
    }else{
      id_dictionary_[req.id_name] = INT_MAX;
      NODELET_WARN("ParameterServerNode: invalid request coming");
      return false;
    }
    
  }
  void IDManagerNode::bindIDCallback(const topic::BindID::Message::ConstPtr &msg){
    NODELET_INFO("id_manager_node: id_name_service_callback");
    NODELET_WARN("You should write canRxCallback in IDManagerNode.");
  }
}// namespace testnode
PLUGINLIB_EXPORT_CLASS(id_manager_node::IDManagerNode, nodelet::Nodelet)
