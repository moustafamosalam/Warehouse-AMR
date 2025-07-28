// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vizanti_msgs:srv/ListLifecycles.idl
// generated code does not contain a copyright notice

#ifndef VIZANTI_MSGS__SRV__DETAIL__LIST_LIFECYCLES__TRAITS_HPP_
#define VIZANTI_MSGS__SRV__DETAIL__LIST_LIFECYCLES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vizanti_msgs/srv/detail/list_lifecycles__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vizanti_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ListLifecycles_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ListLifecycles_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ListLifecycles_Request & msg, bool use_flow_style = false)
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

}  // namespace vizanti_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vizanti_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vizanti_msgs::srv::ListLifecycles_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vizanti_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vizanti_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vizanti_msgs::srv::ListLifecycles_Request & msg)
{
  return vizanti_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vizanti_msgs::srv::ListLifecycles_Request>()
{
  return "vizanti_msgs::srv::ListLifecycles_Request";
}

template<>
inline const char * name<vizanti_msgs::srv::ListLifecycles_Request>()
{
  return "vizanti_msgs/srv/ListLifecycles_Request";
}

template<>
struct has_fixed_size<vizanti_msgs::srv::ListLifecycles_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vizanti_msgs::srv::ListLifecycles_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vizanti_msgs::srv::ListLifecycles_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace vizanti_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ListLifecycles_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: nodes
  {
    if (msg.nodes.size() == 0) {
      out << "nodes: []";
    } else {
      out << "nodes: [";
      size_t pending_items = msg.nodes.size();
      for (auto item : msg.nodes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: states
  {
    if (msg.states.size() == 0) {
      out << "states: []";
    } else {
      out << "states: [";
      size_t pending_items = msg.states.size();
      for (auto item : msg.states) {
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
  const ListLifecycles_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: nodes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.nodes.size() == 0) {
      out << "nodes: []\n";
    } else {
      out << "nodes:\n";
      for (auto item : msg.nodes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.states.size() == 0) {
      out << "states: []\n";
    } else {
      out << "states:\n";
      for (auto item : msg.states) {
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

inline std::string to_yaml(const ListLifecycles_Response & msg, bool use_flow_style = false)
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

}  // namespace vizanti_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vizanti_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vizanti_msgs::srv::ListLifecycles_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vizanti_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vizanti_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vizanti_msgs::srv::ListLifecycles_Response & msg)
{
  return vizanti_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vizanti_msgs::srv::ListLifecycles_Response>()
{
  return "vizanti_msgs::srv::ListLifecycles_Response";
}

template<>
inline const char * name<vizanti_msgs::srv::ListLifecycles_Response>()
{
  return "vizanti_msgs/srv/ListLifecycles_Response";
}

template<>
struct has_fixed_size<vizanti_msgs::srv::ListLifecycles_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vizanti_msgs::srv::ListLifecycles_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vizanti_msgs::srv::ListLifecycles_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vizanti_msgs::srv::ListLifecycles>()
{
  return "vizanti_msgs::srv::ListLifecycles";
}

template<>
inline const char * name<vizanti_msgs::srv::ListLifecycles>()
{
  return "vizanti_msgs/srv/ListLifecycles";
}

template<>
struct has_fixed_size<vizanti_msgs::srv::ListLifecycles>
  : std::integral_constant<
    bool,
    has_fixed_size<vizanti_msgs::srv::ListLifecycles_Request>::value &&
    has_fixed_size<vizanti_msgs::srv::ListLifecycles_Response>::value
  >
{
};

template<>
struct has_bounded_size<vizanti_msgs::srv::ListLifecycles>
  : std::integral_constant<
    bool,
    has_bounded_size<vizanti_msgs::srv::ListLifecycles_Request>::value &&
    has_bounded_size<vizanti_msgs::srv::ListLifecycles_Response>::value
  >
{
};

template<>
struct is_service<vizanti_msgs::srv::ListLifecycles>
  : std::true_type
{
};

template<>
struct is_service_request<vizanti_msgs::srv::ListLifecycles_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vizanti_msgs::srv::ListLifecycles_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VIZANTI_MSGS__SRV__DETAIL__LIST_LIFECYCLES__TRAITS_HPP_
