// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nmea_msgs:msg/Gphdt.idl
// generated code does not contain a copyright notice

#ifndef NMEA_MSGS__MSG__DETAIL__GPHDT__STRUCT_H_
#define NMEA_MSGS__MSG__DETAIL__GPHDT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'message_id'
// Member 'rel_to'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/Gphdt in the package nmea_msgs.
typedef struct nmea_msgs__msg__Gphdt
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String message_id;
  double heading;
  rosidl_runtime_c__String rel_to;
} nmea_msgs__msg__Gphdt;

// Struct for a sequence of nmea_msgs__msg__Gphdt.
typedef struct nmea_msgs__msg__Gphdt__Sequence
{
  nmea_msgs__msg__Gphdt * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nmea_msgs__msg__Gphdt__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NMEA_MSGS__MSG__DETAIL__GPHDT__STRUCT_H_
