#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <boost/array.hpp>

#include <can_plugins/Frame.h>

#include <stdint.h>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <map>

namespace id_manager_node{
  class IDManagerNode: public nodelet::Nodelet
  {
    public:
      void onInit();

    private:
      void canRxCallback(const can_plugins::Frame::ConstPtr &msg);
      void TestTxCallback(const std_msgs::UInt8::ConstPtr &msg);
      bool idNameServiceCallback(const std_msgs::Int32::ConstPtr &msg, const std_msgs::Int32::Ptr &res);

      ros::NodeHandle _nh;
      ros::Publisher _can_tx_pub;
      ros::Subscriber _can_rx_sub;
      ros::ServiceServer _id_name_service;
      std::map<std::string,std::uint32_t> _id_dictionary;
      
  };

  void IDManagerNode::onInit(){
    _nh = getNodeHandle();

    _can_tx_pub	= _nh.advertise<can_plugins::Frame>("can_tx", 1000);
    _can_rx_sub	= _nh.subscribe<can_plugins::Frame>("can_rx", 1000, &IDManagerNode::canRxCallback, this);
//    _id_name_service = _nh.advertiseService("id_name_service",&IDManagerNode::idNameServiceCallback,this);
    NODELET_WARN("I cannnot use advertiseService!!!!!!!!!!!");
    NODELET_INFO("id_manager_node has started.");
  }


  void IDManagerNode::canRxCallback(const can_plugins::Frame::ConstPtr &msg){
    NODELET_INFO("can_rx_callback");
    //TODO
    NODELET_WARN("I don't know what to do with this message. Sorry.");
    NODELET_WARN("You should write canRxCallback in IDManagerNode.");
    //if the data is a id response, set the id to the dictionary
    if(msg->id == 0x700){
      std_msgs::Int32 id_msg;
      id_msg.data = msg->data[0];
      _nh.setParam("id/"+ msg->id,id_msg.data);
    }
  }
  enum class ServiceRequestMessage{
    UpdateAllID,
    getID,
  };

  bool IDManagerNode::idNameServiceCallback(const std_msgs::Int32::ConstPtr &msg, const std_msgs::Int32::Ptr &res){
    NODELET_INFO("id_manager_node: id_name_service_callback");
    NODELET_WARN("I don't know what to do with this message. Sorry.");
    NODELET_WARN("You should write canRxCallback in IDManagerNode.");
    switch (msg->data)
    {
      case static_cast<int>(ServiceRequestMessage::UpdateAllID):
        {
          //make can_tx message for broadcast
          can_plugins::Frame frame;
          frame.id = 0x00;
          frame.dlc = 0;
          frame.data = {};
          _can_tx_pub.publish(frame);

          //if it can success to broadcast, response with true
          res->data = 0;
          return true;
        }
      break;
    
      default:
        {
          NODELET_WARN("IDManagerNode : get invalid service request message");
          //if unknown message, response with false
          res->data = -1;
          return false;
        }
        break;
    }
  }
}// namespace testnode
PLUGINLIB_EXPORT_CLASS(id_manager_node::IDManagerNode, nodelet::Nodelet);
