/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "stepper_driver/interface.hpp"

#include <functional>

namespace stepper_driver {

Interface::Interface(rclcpp::Node *node)
    : remote_actuator::Implementation(node), node_{node} {
  RCLCPP_DEBUG(node_->get_logger(), "Interface::Interface(): started");

  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  node->declare_parameter("stepper_prefix", std::string("/stepper/") +
                                                std::string(node_->get_name()));
  node->get_parameter("stepper_prefix", interface_prefix_);
  auto prefix = get_prefix_();

  rmw_qos_profile_t rmw = {
      .history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 1,
      .reliability =
          rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
      .deadline = {0, 50000000},
      .lifespan = {0, 50000000},
      .liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      .liveliness_lease_duration = {0, 0},
      .avoid_ros_namespace_conventions = false,
  };
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw), rmw);

  param_ppr =
      node->create_publisher<std_msgs::msg::Int32>(prefix + "/param/ppr", qos);

  param_ppr_get = node_->create_service<stepper_driver::srv::ParamPprGet>(
      prefix + "/param/ppr/get",
      std::bind(&Interface::param_ppr_get_handler_, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  param_ppr_set = node_->create_service<stepper_driver::srv::ParamPprSet>(
      prefix + "/param/ppr/set",
      std::bind(&Interface::param_ppr_set_handler_, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);

  RCLCPP_DEBUG(node_->get_logger(), "Interface::Interface(): ended");
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

}  // namespace stepper_driver
