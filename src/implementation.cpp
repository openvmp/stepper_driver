/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_stepper_driver/implementation.hpp"

#include <functional>

namespace remote_stepper_driver {

Implementation::Implementation(rclcpp::Node *node)
    : remote_actuator::Implementation(node), node_{node} {
  RCLCPP_DEBUG(node_->get_logger(),
               "Implementation::Implementation(): started");

  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  node->declare_parameter(
      "stepper_prefix",
      std::string("/actuator/") + std::string(node_->get_name()) + "/stepper");
  node->get_parameter("stepper_prefix", interface_prefix_);
  auto prefix = get_prefix_();

  param_ppr =
      node->create_publisher<std_msgs::msg::Int32>(prefix + "/param/ppr", 10);

  param_ppr_get =
      node_->create_service<remote_stepper_driver::srv::ParamPprGet>(
          prefix + "/param/ppr/get",
          std::bind(&Implementation::param_ppr_get_handler_, this,
                    std::placeholders::_1, std::placeholders::_2),
          ::rmw_qos_profile_default, callback_group_);
  param_ppr_set =
      node_->create_service<remote_stepper_driver::srv::ParamPprSet>(
          prefix + "/param/ppr/set",
          std::bind(&Implementation::param_ppr_set_handler_, this,
                    std::placeholders::_1, std::placeholders::_2),
          ::rmw_qos_profile_default, callback_group_);

  RCLCPP_DEBUG(node_->get_logger(), "Implementation::Implementation(): ended");
}

std::string Implementation::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

}  // namespace remote_stepper_driver
