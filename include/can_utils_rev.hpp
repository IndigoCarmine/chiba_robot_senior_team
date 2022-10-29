//modify the following code: can_utils.hpp in can_plugins pkg
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

#include <boost/array.hpp>
#include <can_plugins/Frame.h>

namespace can_utils{
  constexpr unsigned int can_mtu = 8;

  enum Endian{
    LITTLE,
    BIG
  };

  // Validness of the following code depends on the assumption that can_plugins::Frame::_data_type is uint8_t[can_mtu] and uint8_t is unsigned char.
  static_assert(std::is_same<uint8_t, unsigned char>::value, "This code is assuming that uint8_t is unsigned char.");




  //This function is used to extract the data from the can message.
  //Don't use this function directly. Use makeFrame.
  template <typename T>
    inline can_plugins::Frame::_data_type pack(const T data, Endian endian = Endian::BIG)
  {
    // assertion.
    static_assert(std::is_scalar<T>::value, "T is not scalar type.");
    static_assert(sizeof(T) <= can_mtu, "size of T is larger than can_mtu.");

    can_plugins::Frame::_data_type buffer{};  // It is not necessary to value-initialize this for the sake of transmission. But it will come in handy for debugging.
    std::memcpy(buffer.c_array(), &data, sizeof(T));

    if(endian == Endian::LITTLE){
      std::reverse(buffer.begin(), buffer.begin() + sizeof(T));
    }else if(endian == Endian::BIG){
      //do nothing
    }else{
      throw std::invalid_argument("Endian is invalid.");
    }
    return buffer;
  }

  //This function is used to pack the data into the can message.
  template<typename T>
  inline T unpack(can_plugins::Frame::_data_type buffer, Endian endian = Endian::BIG)
  {
    // assertion.
    static_assert(std::is_scalar<T>::value, "T is not scalar type.");
    static_assert(sizeof(T) <= can_mtu, "size of T is larger than can_mtu.");

    if(endian == Endian::LITTLE){
      std::reverse(buffer.begin(), buffer.begin() + sizeof(T));
    }else if(endian == Endian::BIG){
      //do nothing
    }else{
      throw std::runtime_error("endian is not supported.");
    }

    T data;
    std::memcpy(&data, buffer.c_array(), sizeof(T));
  
    return data;
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
    frame.data = pack(data);
    return frame;
  }


  //it is implanted in shirasu_fw/MoterCtrl.cpp
  enum class Command : uint8_t {
    shutdown,
    recover,
    home,
    get_status,
    recover_current,
    recover_velocity,
    recover_position
  };

  //overload for Command
  template<typename T>
  static can_plugins::Frame makeFrame(const uint16_t id, Command cmd)
  {
    //when you use the id, it works as the command if moter.
      return makeFrame(id,(uint8_t)cmd);
  }
}