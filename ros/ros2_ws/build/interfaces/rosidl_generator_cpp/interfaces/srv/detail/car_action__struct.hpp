// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:srv/CarAction.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__CAR_ACTION__STRUCT_HPP_
#define INTERFACES__SRV__DETAIL__CAR_ACTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__CarAction_Request __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__CarAction_Request __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CarAction_Request_
{
  using Type = CarAction_Request_<ContainerAllocator>;

  explicit CarAction_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steer = 0.0f;
      this->speed = 0.0f;
    }
  }

  explicit CarAction_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steer = 0.0f;
      this->speed = 0.0f;
    }
  }

  // field types and members
  using _steer_type =
    float;
  _steer_type steer;
  using _speed_type =
    float;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__steer(
    const float & _arg)
  {
    this->steer = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::CarAction_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::CarAction_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::CarAction_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::CarAction_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::CarAction_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::CarAction_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::CarAction_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::CarAction_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::CarAction_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::CarAction_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__CarAction_Request
    std::shared_ptr<interfaces::srv::CarAction_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__CarAction_Request
    std::shared_ptr<interfaces::srv::CarAction_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarAction_Request_ & other) const
  {
    if (this->steer != other.steer) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarAction_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarAction_Request_

// alias to use template instance with default allocator
using CarAction_Request =
  interfaces::srv::CarAction_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__CarAction_Response __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__CarAction_Response __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CarAction_Response_
{
  using Type = CarAction_Response_<ContainerAllocator>;

  explicit CarAction_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = "";
    }
  }

  explicit CarAction_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : x(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = "";
    }
  }

  // field types and members
  using _x_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _x_type x;

  // setters for named parameter idiom
  Type & set__x(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->x = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::CarAction_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::CarAction_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::CarAction_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::CarAction_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::CarAction_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::CarAction_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::CarAction_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::CarAction_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::CarAction_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::CarAction_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__CarAction_Response
    std::shared_ptr<interfaces::srv::CarAction_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__CarAction_Response
    std::shared_ptr<interfaces::srv::CarAction_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarAction_Response_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarAction_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarAction_Response_

// alias to use template instance with default allocator
using CarAction_Response =
  interfaces::srv::CarAction_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces

namespace interfaces
{

namespace srv
{

struct CarAction
{
  using Request = interfaces::srv::CarAction_Request;
  using Response = interfaces::srv::CarAction_Response;
};

}  // namespace srv

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__CAR_ACTION__STRUCT_HPP_
