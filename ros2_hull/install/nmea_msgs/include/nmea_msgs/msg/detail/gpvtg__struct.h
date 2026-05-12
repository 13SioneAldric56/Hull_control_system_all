// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nmea_msgs:msg/Gpvtg.idl
// generated code does not contain a copyright notice

#ifndef NMEA_MSGS__MSG__DETAIL__GPVTG__STRUCT_H_
#define NMEA_MSGS__MSG__DETAIL__GPVTG__STRUCT_H_

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
// Member 'track_t_ref'
// Member 'track_m_ref'
// Member 'speed_n_unit'
// Member 'speed_k_unit'
// Member 'mode_indicator'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/Gpvtg in the package nmea_msgs.
typedef struct nmea_msgs__msg__Gpvtg
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String message_id;
  float track_t;
  rosidl_runtime_c__String track_t_ref;
  float track_m;
  rosidl_runtime_c__String track_m_ref;
  float speed_n;
  rosidl_runtime_c__String speed_n_unit;
  float speed_k;
  rosidl_runtime_c__String speed_k_unit;
  rosidl_runtime_c__String mode_indicator;
} nmea_msgs__msg__Gpvtg;

// Struct for a sequence of nmea_msgs__msg__Gpvtg.
typedef struct nmea_msgs__msg__Gpvtg__Sequence
{
  nmea_msgs__msg__Gpvtg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nmea_msgs__msg__Gpvtg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NMEA_MSGS__MSG__DETAIL__GPVTG__STRUCT_H_
