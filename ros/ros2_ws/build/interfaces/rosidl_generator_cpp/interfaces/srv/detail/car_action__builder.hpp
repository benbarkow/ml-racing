// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/CarAction.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__CAR_ACTION__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__CAR_ACTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/car_action__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_CarAction_Request_speed
{
public:
  explicit Init_CarAction_Request_speed(::interfaces::srv::CarAction_Request & msg)
  : msg_(msg)
  {}
  ::interfaces::srv::CarAction_Request speed(::interfaces::srv::CarAction_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::CarAction_Request msg_;
};

class Init_CarAction_Request_steer
{
public:
  Init_CarAction_Request_steer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarAction_Request_speed steer(::interfaces::srv::CarAction_Request::_steer_type arg)
  {
    msg_.steer = std::move(arg);
    return Init_CarAction_Request_speed(msg_);
  }

private:
  ::interfaces::srv::CarAction_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::CarAction_Request>()
{
  return interfaces::srv::builder::Init_CarAction_Request_steer();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_CarAction_Response_x
{
public:
  Init_CarAction_Response_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::CarAction_Response x(::interfaces::srv::CarAction_Response::_x_type arg)
  {
    msg_.x = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::CarAction_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::CarAction_Response>()
{
  return interfaces::srv::builder::Init_CarAction_Response_x();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__CAR_ACTION__BUILDER_HPP_
