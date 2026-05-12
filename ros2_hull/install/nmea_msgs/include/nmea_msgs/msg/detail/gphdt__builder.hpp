// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nmea_msgs:msg/Gphdt.idl
// generated code does not contain a copyright notice

#ifndef NMEA_MSGS__MSG__DETAIL__GPHDT__BUILDER_HPP_
#define NMEA_MSGS__MSG__DETAIL__GPHDT__BUILDER_HPP_

#include "nmea_msgs/msg/detail/gphdt__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace nmea_msgs
{

namespace msg
{

namespace builder
{

class Init_Gphdt_rel_to
{
public:
  explicit Init_Gphdt_rel_to(::nmea_msgs::msg::Gphdt & msg)
  : msg_(msg)
  {}
  ::nmea_msgs::msg::Gphdt rel_to(::nmea_msgs::msg::Gphdt::_rel_to_type arg)
  {
    msg_.rel_to = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nmea_msgs::msg::Gphdt msg_;
};

class Init_Gphdt_heading
{
public:
  explicit Init_Gphdt_heading(::nmea_msgs::msg::Gphdt & msg)
  : msg_(msg)
  {}
  Init_Gphdt_rel_to heading(::nmea_msgs::msg::Gphdt::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_Gphdt_rel_to(msg_);
  }

private:
  ::nmea_msgs::msg::Gphdt msg_;
};

class Init_Gphdt_message_id
{
public:
  explicit Init_Gphdt_message_id(::nmea_msgs::msg::Gphdt & msg)
  : msg_(msg)
  {}
  Init_Gphdt_heading message_id(::nmea_msgs::msg::Gphdt::_message_id_type arg)
  {
    msg_.message_id = std::move(arg);
    return Init_Gphdt_heading(msg_);
  }

private:
  ::nmea_msgs::msg::Gphdt msg_;
};

class Init_Gphdt_header
{
public:
  Init_Gphdt_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gphdt_message_id header(::nmea_msgs::msg::Gphdt::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Gphdt_message_id(msg_);
  }

private:
  ::nmea_msgs::msg::Gphdt msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nmea_msgs::msg::Gphdt>()
{
  return nmea_msgs::msg::builder::Init_Gphdt_header();
}

}  // namespace nmea_msgs

#endif  // NMEA_MSGS__MSG__DETAIL__GPHDT__BUILDER_HPP_
