#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <boost/array.hpp>

#include <can_plugins/Frame.h>
#include <can_utils_rev.hpp>
#include "common_settings.hpp"

#include <stdint.h>
#include <string>
#include <map>


#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>




namespace id_manager_node{
  class IDManagerNode: public nodelet::Nodelet
  {
    public:
      void onInit();

    private:
      void canRxCallback(const can_plugins::Frame::ConstPtr &msg);
      void TestTxCallback(const std_msgs::UInt8::ConstPtr &msg);
      bool idNameServiceCallback(const std_msgs::Int32::ConstPtr &msg, const std_msgs::Int32::Ptr &res);

      ros::NodeHandle nodehandle_;
      ros::Publisher can_tx_pub_;
      ros::Subscriber can_rx_sub_;
      ros::ServiceServer id_name_service_;
      std::map<std::string,std::uint32_t> id_dictionary_;
      
  };

  void IDManagerNode::onInit(){
    nodehandle_ = getMTNodeHandle();

    can_tx_pub_	= nodehandle_.advertise<common_settings::topic::CanTx::Message>(common_settings::topic::CanTx::name, 1);
    can_rx_sub_	= nodehandle_.subscribe<common_settings::topic::CanRx::Message>(common_settings::topic::CanRx::name, 1, &IDManagerNode::canRxCallback, this);
//    _id_name_service = _nh.advertiseService("id_name_service",&IDManagerNode::idNameServiceCallback,this);
    NODELET_WARN("I cannnot use advertiseService!!!!!!!!!!!");
    NODELET_INFO("IDManagerNode has started.");
  }


  void IDManagerNode::canRxCallback(const can_plugins::Frame::ConstPtr &msg){
    //TODO
    NODELET_WARN("I don't know what to do with this message. Sorry.");
    NODELET_WARN("You should write canRxCallback in IDManagerNode.");
    //if the data is a id response, set the id to the dictionary
    if(msg->id == 0x700){
      std_msgs::Int32 id_msg;
      id_msg.data = msg->data[0];
      nodehandle_.setParam("id/"+ msg->id,id_msg.data);
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
          can_tx_pub_.publish(frame);

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
PLUGINLIB_EXPORT_CLASS(id_manager_node::IDManagerNode, nodelet::Nodelet)
