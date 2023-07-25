// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:srv/TrackingData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__TRACKING_DATA__STRUCT_H_
#define INTERFACES__SRV__DETAIL__TRACKING_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'x'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TrackingData in the package interfaces.
typedef struct interfaces__srv__TrackingData_Request
{
  rosidl_runtime_c__String x;
} interfaces__srv__TrackingData_Request;

// Struct for a sequence of interfaces__srv__TrackingData_Request.
typedef struct interfaces__srv__TrackingData_Request__Sequence
{
  interfaces__srv__TrackingData_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__TrackingData_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'positioncar'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/TrackingData in the package interfaces.
typedef struct interfaces__srv__TrackingData_Response
{
  float speedx;
  float speedy;
  rosidl_runtime_c__float__Sequence positioncar;
} interfaces__srv__TrackingData_Response;

// Struct for a sequence of interfaces__srv__TrackingData_Response.
typedef struct interfaces__srv__TrackingData_Response__Sequence
{
  interfaces__srv__TrackingData_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__TrackingData_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__SRV__DETAIL__TRACKING_DATA__STRUCT_H_
