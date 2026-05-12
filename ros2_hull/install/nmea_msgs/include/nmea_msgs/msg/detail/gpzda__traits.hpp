// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nmea_msgs:msg/Gpzda.idl
// generated code does not contain a copyright notice

#ifndef NMEA_MSGS__MSG__DETAIL__GPZDA__TRAITS_HPP_
#define NMEA_MSGS__MSG__DETAIL__GPZDA__TRAITS_HPP_

#include "nmea_msgs/msg/detail/gpzda__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nmea_msgs::msg::Gpzda>()
{
  return "nmea_msgs::msg::Gpzda";
}

template<>
inline const char * name<nmea_msgs::msg::Gpzda>()
{
  return "nmea_msgs/msg/Gpzda";
}

template<>
struct has_fixed_size<nmea_msgs::msg::Gpzda>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nmea_msgs::msg::Gpzda>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<nmea_msgs::msg::Gpzda>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NMEA_MSGS__MSG__DETAIL__GPZDA__TRAITS_HPP_
