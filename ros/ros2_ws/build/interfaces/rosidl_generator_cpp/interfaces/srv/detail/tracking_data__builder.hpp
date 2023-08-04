// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/TrackingData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__TRACKING_DATA__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__TRACKING_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/tracking_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_TrackingData_Request_x
{
public:
  Init_TrackingData_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::TrackingData_Request x(::interfaces::srv::TrackingData_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::TrackingData_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::TrackingData_Request>()
{
  return interfaces::srv::builder::Init_TrackingData_Request_x();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_TrackingData_Response_positioncar
{
public:
  explicit Init_TrackingData_Response_positioncar(::interfaces::srv::TrackingData_Response & msg)
  : msg_(msg)
  {}
  ::interfaces::srv::TrackingData_Response positioncar(::interfaces::srv::TrackingData_Response::_positioncar_type arg)
  {
    msg_.positioncar = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::TrackingData_Response msg_;
};

class Init_TrackingData_Response_speedy
{
public:
  explicit Init_TrackingData_Response_speedy(::interfaces::srv::TrackingData_Response & msg)
  : msg_(msg)
  {}
  Init_TrackingData_Response_positioncar speedy(::interfaces::srv::TrackingData_Response::_speedy_type arg)
  {
    msg_.speedy = std::move(arg);
    return Init_TrackingData_Response_positioncar(msg_);
  }

private:
  ::interfaces::srv::TrackingData_Response msg_;
};

class Init_TrackingData_Response_speedx
{
public:
  Init_TrackingData_Response_speedx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackingData_Response_speedy speedx(::interfaces::srv::TrackingData_Response::_speedx_type arg)
  {
    msg_.speedx = std::move(arg);
    return Init_TrackingData_Response_speedy(msg_);
  }

private:
  ::interfaces::srv::TrackingData_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::TrackingData_Response>()
{
  return interfaces::srv::builder::Init_TrackingData_Response_speedx();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__TRACKING_DATA__BUILDER_HPP_
