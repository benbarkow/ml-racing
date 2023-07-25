// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:srv/TrackingData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__TRACKING_DATA__TRAITS_HPP_
#define INTERFACES__SRV__DETAIL__TRACKING_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/srv/detail/tracking_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const TrackingData_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrackingData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrackingData_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::TrackingData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::TrackingData_Request & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::TrackingData_Request>()
{
  return "interfaces::srv::TrackingData_Request";
}

template<>
inline const char * name<interfaces::srv::TrackingData_Request>()
{
  return "interfaces/srv/TrackingData_Request";
}

template<>
struct has_fixed_size<interfaces::srv::TrackingData_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::srv::TrackingData_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::srv::TrackingData_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const TrackingData_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: speedx
  {
    out << "speedx: ";
    rosidl_generator_traits::value_to_yaml(msg.speedx, out);
    out << ", ";
  }

  // member: speedy
  {
    out << "speedy: ";
    rosidl_generator_traits::value_to_yaml(msg.speedy, out);
    out << ", ";
  }

  // member: positioncar
  {
    if (msg.positioncar.size() == 0) {
      out << "positioncar: []";
    } else {
      out << "positioncar: [";
      size_t pending_items = msg.positioncar.size();
      for (auto item : msg.positioncar) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrackingData_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: speedx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speedx: ";
    rosidl_generator_traits::value_to_yaml(msg.speedx, out);
    out << "\n";
  }

  // member: speedy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speedy: ";
    rosidl_generator_traits::value_to_yaml(msg.speedy, out);
    out << "\n";
  }

  // member: positioncar
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.positioncar.size() == 0) {
      out << "positioncar: []\n";
    } else {
      out << "positioncar:\n";
      for (auto item : msg.positioncar) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrackingData_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::TrackingData_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::TrackingData_Response & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::TrackingData_Response>()
{
  return "interfaces::srv::TrackingData_Response";
}

template<>
inline const char * name<interfaces::srv::TrackingData_Response>()
{
  return "interfaces/srv/TrackingData_Response";
}

template<>
struct has_fixed_size<interfaces::srv::TrackingData_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::srv::TrackingData_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::srv::TrackingData_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interfaces::srv::TrackingData>()
{
  return "interfaces::srv::TrackingData";
}

template<>
inline const char * name<interfaces::srv::TrackingData>()
{
  return "interfaces/srv/TrackingData";
}

template<>
struct has_fixed_size<interfaces::srv::TrackingData>
  : std::integral_constant<
    bool,
    has_fixed_size<interfaces::srv::TrackingData_Request>::value &&
    has_fixed_size<interfaces::srv::TrackingData_Response>::value
  >
{
};

template<>
struct has_bounded_size<interfaces::srv::TrackingData>
  : std::integral_constant<
    bool,
    has_bounded_size<interfaces::srv::TrackingData_Request>::value &&
    has_bounded_size<interfaces::srv::TrackingData_Response>::value
  >
{
};

template<>
struct is_service<interfaces::srv::TrackingData>
  : std::true_type
{
};

template<>
struct is_service_request<interfaces::srv::TrackingData_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interfaces::srv::TrackingData_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__SRV__DETAIL__TRACKING_DATA__TRAITS_HPP_
