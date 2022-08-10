/*
 * mr1_can.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <boost/array.hpp>

#include <can_plugins/Frame.h>
#include "can_utils.hpp"

#include <stdint.h>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <map>

namespace id_manager_node{
  class IDManagerNode: public nodelet::Nodelet
  {
    public:
      virtual void onInit();

    private:
      void canRxCallback(const can_plugins::Frame::ConstPtr &msg);

      void TestTxCallback(const std_msgs::UInt8::ConstPtr &msg);
      void idNameServiceCallback(const std_msgs::String::ConstPtr &msg);
      template<typename T>
        void sendData(const uint16_t id, const T data);

      ros::NodeHandle _nh;
      ros::NodeHandle pnh;
      ros::Publisher _can_tx_pub;
      ros::Subscriber _can_rx_sub;
      ros::ServiceServer _id_name_service;
      std::map<std::string,std::uint32_t> _id_dictionary;
      
  };

  void IDManagerNode::onInit(){
    _nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    _can_tx_pub	= _nh.advertise<can_plugins::Frame>("can_tx", 1000);
    _can_rx_sub	= _nh.subscribe<can_plugins::Frame>("can_rx", 1000, &IDManagerNode::canRxCallback, this);
    _id_name_service = _nh.advertiseService<std_msgs::String::ConstPtr>("id_name_service",&IDManagerNode::idNameServiceCallback);
    NODELET_INFO("id_manager_node has started.");

    //TODO We should get CAN ID and name.
    _id_dictionary = {{"test",1}};
  }


  void IDManagerNode::canRxCallback(const can_plugins::Frame::ConstPtr &msg){

  }
  void IDManagerNode::idNameServiceCallback(const std_msgs::String::ConstPtr &msg ){
        
  }
}// namespace testnode
PLUGINLIB_EXPORT_CLASS(id_manager_node::IDManagerNode, nodelet::Nodelet);
