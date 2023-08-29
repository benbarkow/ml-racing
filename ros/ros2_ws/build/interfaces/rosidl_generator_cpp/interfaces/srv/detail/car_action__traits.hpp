// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:srv/CarAction.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__CAR_ACTION__TRAITS_HPP_
#define INTERFACES__SRV__DETAIL__CAR_ACTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/srv/detail/car_action__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CarAction_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: steer
  {
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarAction_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: steer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarAction_Request & msg, bool use_flow_style = false)
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
  const interfaces::srv::CarAction_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::CarAction_Request & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::CarAction_Request>()
{
  return "interfaces::srv::CarAction_Request";
}

template<>
inline const char * name<interfaces::srv::CarAction_Request>()
{
  return "interfaces/srv/CarAction_Request";
}

template<>
struct has_fixed_size<interfaces::srv::CarAction_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::srv::CarAction_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::srv::CarAction_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CarAction_Response & msg,
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
  const CarAction_Response & msg,
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

inline std::string to_yaml(const CarAction_Response & msg, bool use_flow_style = false)
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
  const interfaces::srv::CarAction_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::CarAction_Response & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::CarAction_Response>()
{
  return "interfaces::srv::CarAction_Response";
}

template<>
inline const char * name<interfaces::srv::CarAction_Response>()
{
  return "interfaces/srv/CarAction_Response";
}

template<>
struct has_fixed_size<interfaces::srv::CarAction_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::srv::CarAction_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::srv::CarAction_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interfaces::srv::CarAction>()
{
  return "interfaces::srv::CarAction";
}

template<>
inline const char * name<interfaces::srv::CarAction>()
{
  return "interfaces/srv/CarAction";
}

template<>
struct has_fixed_size<interfaces::srv::CarAction>
  : std::integral_constant<
    bool,
    has_fixed_size<interfaces::srv::CarAction_Request>::value &&
    has_fixed_size<interfaces::srv::CarAction_Response>::value
  >
{
};

template<>
struct has_bounded_size<interfaces::srv::CarAction>
  : std::integral_constant<
    bool,
    has_bounded_size<interfaces::srv::CarAction_Request>::value &&
    has_bounded_size<interfaces::srv::CarAction_Response>::value
  >
{
};

template<>
struct is_service<interfaces::srv::CarAction>
  : std::true_type
{
};

template<>
struct is_service_request<interfaces::srv::CarAction_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interfaces::srv::CarAction_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__SRV__DETAIL__CAR_ACTION__TRAITS_HPP_
