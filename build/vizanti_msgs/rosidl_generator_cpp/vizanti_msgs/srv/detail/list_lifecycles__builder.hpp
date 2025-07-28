// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vizanti_msgs:srv/ListLifecycles.idl
// generated code does not contain a copyright notice

#ifndef VIZANTI_MSGS__SRV__DETAIL__LIST_LIFECYCLES__BUILDER_HPP_
#define VIZANTI_MSGS__SRV__DETAIL__LIST_LIFECYCLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vizanti_msgs/srv/detail/list_lifecycles__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vizanti_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vizanti_msgs::srv::ListLifecycles_Request>()
{
  return ::vizanti_msgs::srv::ListLifecycles_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace vizanti_msgs


namespace vizanti_msgs
{

namespace srv
{

namespace builder
{

class Init_ListLifecycles_Response_states
{
public:
  explicit Init_ListLifecycles_Response_states(::vizanti_msgs::srv::ListLifecycles_Response & msg)
  : msg_(msg)
  {}
  ::vizanti_msgs::srv::ListLifecycles_Response states(::vizanti_msgs::srv::ListLifecycles_Response::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vizanti_msgs::srv::ListLifecycles_Response msg_;
};

class Init_ListLifecycles_Response_nodes
{
public:
  Init_ListLifecycles_Response_nodes()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ListLifecycles_Response_states nodes(::vizanti_msgs::srv::ListLifecycles_Response::_nodes_type arg)
  {
    msg_.nodes = std::move(arg);
    return Init_ListLifecycles_Response_states(msg_);
  }

private:
  ::vizanti_msgs::srv::ListLifecycles_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vizanti_msgs::srv::ListLifecycles_Response>()
{
  return vizanti_msgs::srv::builder::Init_ListLifecycles_Response_nodes();
}

}  // namespace vizanti_msgs

#endif  // VIZANTI_MSGS__SRV__DETAIL__LIST_LIFECYCLES__BUILDER_HPP_
