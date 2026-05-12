// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nmea_msgs:msg/GpgsvSatellite.idl
// generated code does not contain a copyright notice

#ifndef NMEA_MSGS__MSG__DETAIL__GPGSV_SATELLITE__TRAITS_HPP_
#define NMEA_MSGS__MSG__DETAIL__GPGSV_SATELLITE__TRAITS_HPP_

#include "nmea_msgs/msg/detail/gpgsv_satellite__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nmea_msgs::msg::GpgsvSatellite>()
{
  return "nmea_msgs::msg::GpgsvSatellite";
}

template<>
inline const char * name<nmea_msgs::msg::GpgsvSatellite>()
{
  return "nmea_msgs/msg/GpgsvSatellite";
}

template<>
struct has_fixed_size<nmea_msgs::msg::GpgsvSatellite>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<nmea_msgs::msg::GpgsvSatellite>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<nmea_msgs::msg::GpgsvSatellite>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NMEA_MSGS__MSG__DETAIL__GPGSV_SATELLITE__TRAITS_HPP_
