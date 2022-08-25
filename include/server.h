#ifndef __SERVER_H__
#define __SERVER_H__

#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <chiba_robot_senior_team/ParameterConfig.h>
/**
 * @todo Add diagnostics.
 */

using namespace chiba_robot_senior_team;

namespace dynamic_reconfigure
{
/**
 * Keeps track of the reconfigure callback function.
 */
class Server
{
public:
  void onInit(ros::NodeHandle &nh)
  {
    node_handle_ = nh;
    own_mutex_warn_ = true;
    init();
  }


  typedef boost::function<void(ParameterConfig &, uint32_t level)> CallbackType;

  void setCallback(const CallbackType &callback)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    callback_ = callback;
    callCallback(config_, ~0); // At startup we need to load the configuration with all level bits set. (Everything has changed.)
    updateConfigInternal(config_);
  }

  void clearCallback()
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    callback_.clear();
  }

  void updateConfig(const ParameterConfig &config)
  {
    if (own_mutex_warn_)
    {
      ROS_WARN("updateConfig() called on a dynamic_reconfigure::Server that provides its own mutex. This can lead to deadlocks if updateConfig() is called during an update. Providing a mutex to the constructor is highly recommended in this case. Please forward this message to the node author.");
      own_mutex_warn_ = false;
    }
    updateConfigInternal(config);
  }


  void getConfigMax(ParameterConfig &config)
  {
    config = max_;
  }

  void getConfigMin(ParameterConfig &config)
  {
    config = min_;
  }

  void getConfigDefault(ParameterConfig &config)
  {
    config = default_;
  }

  void setConfigMax(const ParameterConfig &config)
  {
    max_ = config;
    PublishDescription();
  }

  void setConfigMin(const ParameterConfig &config)
  {
    min_ = config;
    PublishDescription();
  }

  void setConfigDefault(const ParameterConfig &config)
  {
    default_ = config;
    PublishDescription();
  }


private:
  ros::NodeHandle node_handle_;
  ros::ServiceServer set_service_;
  ros::Publisher update_pub_;
  ros::Publisher descr_pub_;
  CallbackType callback_;
  ParameterConfig config_;
  ParameterConfig min_;
  ParameterConfig max_;
  ParameterConfig default_;
  boost::recursive_mutex &mutex_;
  boost::recursive_mutex own_mutex_; // Used only if an external one isn't specified.
  bool own_mutex_warn_;



  void PublishDescription()
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    //Copy over min_ max_ default_
    dynamic_reconfigure::ConfigDescription description_message = ParameterConfig::__getDescriptionMessage__();

    max_.__toMessage__(description_message.max, ParameterConfig::__getParamDescriptions__(),ParameterConfig::__getGroupDescriptions__());
    min_.__toMessage__(description_message.min,ParameterConfig::__getParamDescriptions__(),ParameterConfig::__getGroupDescriptions__());
    default_.__toMessage__(description_message.dflt,ParameterConfig::__getParamDescriptions__(),ParameterConfig::__getGroupDescriptions__());

    //Publish description
    descr_pub_.publish(description_message);
  }

  void init()
  {
    //Grab copys of the data from the config files.  These are declared in the generated config file.
    min_ = ParameterConfig::__getMin__();
    max_ = ParameterConfig::__getMax__();
    default_ = ParameterConfig::__getDefault__();

    boost::recursive_mutex::scoped_lock lock(mutex_);
    set_service_ = node_handle_.advertiseService("set_parameters",
        &Server::setConfigCallback, this);

    descr_pub_ = node_handle_.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
    descr_pub_.publish(ParameterConfig::__getDescriptionMessage__());

    update_pub_ = node_handle_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);
    ParameterConfig init_config = ParameterConfig::__getDefault__();
    init_config.__fromServer__(node_handle_);
    init_config.__clamp__();
    updateConfigInternal(init_config);
  }

  void callCallback(ParameterConfig &config, int level)
  {
    if (callback_) // At startup we need to load the configuration with all level bits set. (Everything has changed.)
      try {
        callback_(config, level);
      }
      catch (std::exception &e)
      {
        ROS_WARN("Reconfigure callback failed with exception %s: ", e.what());
      }
      catch (...)
      {
        ROS_WARN("Reconfigure callback failed with unprintable exception.");
      }
    else
      ROS_DEBUG("setCallback did not call callback because it was zero."); /// @todo kill this line.
  }

  bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
          dynamic_reconfigure::Reconfigure::Response &rsp)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);

    ParameterConfig new_config = config_;
    new_config.__fromMessage__(req.config);
    new_config.__clamp__();
    uint32_t level = config_.__level__(new_config);

    callCallback(new_config, level);

    updateConfigInternal(new_config);
    new_config.__toMessage__(rsp.config);
    return true;
  }

  void updateConfigInternal(const ParameterConfig &config)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    config_ = config;
    config_.__toServer__(node_handle_);
    dynamic_reconfigure::Config msg;
    config_.__toMessage__(msg);
    update_pub_.publish(msg);
  }
};

}
#endif
