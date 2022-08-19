//modify the following code: can_utils.hpp in can_plugins pkg

#include <boost/array.hpp>
#include <can_plugins/Frame.h>

#define CAN_MTU 8

namespace can_utils{
  //Don't use this union directly. Use the functions below.
  template<typename T>
  union _Encapsulator
  {
    T data;
    uint64_t i;
  };

  //This function is used to extract the data from the can message.
  template <typename T>
  static void canUnpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
  {
    _Encapsulator<T> _e;
    for(int i = 0; i < sizeof(T); i++)
    {
      _e.i = (_e.i << 8) | (uint64_t)(buf[i]);
    }
    data = _e.data;
  }

  //This function is used to pack the data into the can message.
  //Don't use this function directly. Use makeFrame.
  template<typename T>
  static void canPack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
  {
    _Encapsulator<T> _e;
    _e.data = data;
    for(int i = sizeof(T); i > 0;)
    {
      i--;
      buf[i] = _e.i & 0xff;
      _e.i >>= 8;
    }
  }

  //This function is used to encapsulate the data into a can frame.
  template<typename T>
  static can_plugins::Frame makeFrame(const uint16_t id, const T data)
  {
    can_plugins::Frame frame;
    //when you want to use SetTarget, you need to set the id to the following id.
    frame.id = id+1;

    //I don't know what is the meaning of theses parameters well but I think they are important.
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;

    frame.dlc = sizeof(T);
    canPack(frame.data, data);
    return frame;
  }


  //it is implanted in shirasu_fw/MoterCtrl.cpp
  enum class Comand : uint8_t {
    shutdown,
    recover,
    home,
    get_status,
    recover_current,
    recover_velocity,
    recover_position
  };

  //overload for Comand
  template<typename T>
  static can_plugins::Frame makeFrame(const uint16_t id,Comand cmd)
  {
    //when you use the id, it works as the command if moter.
      return makeFrame(id,(uint8_t)cmd);
  }
}