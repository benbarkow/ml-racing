// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:srv/TrackingData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__TRACKING_DATA__STRUCT_HPP_
#define INTERFACES__SRV__DETAIL__TRACKING_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__TrackingData_Request __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__TrackingData_Request __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TrackingData_Request_
{
  using Type = TrackingData_Request_<ContainerAllocator>;

  explicit TrackingData_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = "";
    }
  }

  explicit TrackingData_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    interfaces::srv::TrackingData_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::TrackingData_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::TrackingData_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::TrackingData_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::TrackingData_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::TrackingData_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::TrackingData_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::TrackingData_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::TrackingData_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::TrackingData_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__TrackingData_Request
    std::shared_ptr<interfaces::srv::TrackingData_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__TrackingData_Request
    std::shared_ptr<interfaces::srv::TrackingData_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackingData_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackingData_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackingData_Request_

// alias to use template instance with default allocator
using TrackingData_Request =
  interfaces::srv::TrackingData_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__TrackingData_Response __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__TrackingData_Response __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TrackingData_Response_
{
  using Type = TrackingData_Response_<ContainerAllocator>;

  explicit TrackingData_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speedx = 0.0f;
      this->speedy = 0.0f;
    }
  }

  explicit TrackingData_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speedx = 0.0f;
      this->speedy = 0.0f;
    }
  }

  // field types and members
  using _speedx_type =
    float;
  _speedx_type speedx;
  using _speedy_type =
    float;
  _speedy_type speedy;
  using _positioncar_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _positioncar_type positioncar;

  // setters for named parameter idiom
  Type & set__speedx(
    const float & _arg)
  {
    this->speedx = _arg;
    return *this;
  }
  Type & set__speedy(
    const float & _arg)
  {
    this->speedy = _arg;
    return *this;
  }
  Type & set__positioncar(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->positioncar = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::TrackingData_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::TrackingData_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::TrackingData_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::TrackingData_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::TrackingData_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::TrackingData_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::TrackingData_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::TrackingData_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::TrackingData_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::TrackingData_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__TrackingData_Response
    std::shared_ptr<interfaces::srv::TrackingData_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__TrackingData_Response
    std::shared_ptr<interfaces::srv::TrackingData_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackingData_Response_ & other) const
  {
    if (this->speedx != other.speedx) {
      return false;
    }
    if (this->speedy != other.speedy) {
      return false;
    }
    if (this->positioncar != other.positioncar) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackingData_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackingData_Response_

// alias to use template instance with default allocator
using TrackingData_Response =
  interfaces::srv::TrackingData_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces

namespace interfaces
{

namespace srv
{

struct TrackingData
{
  using Request = interfaces::srv::TrackingData_Request;
  using Response = interfaces::srv::TrackingData_Response;
};

}  // namespace srv

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__TRACKING_DATA__STRUCT_HPP_
