/*
 * Copyright 2022 OpenVMP Authors
 *
 * Licensed under HIPPOCRATIC LICENSE Version 3.0.
 * Generated using
 * https://firstdonoharm.dev/version/3/0/bds-bod-cl-eco-ffd-media-mil-soc-sup-sv.md
 * See https://github.com/openvmp/openvmp/blob/main/docs/License.md for more
 * details.
 *
 */

#include "stepper_driver/interface.hpp"

#include <functional>

namespace stepper_driver {

StepperDriverInterface::StepperDriverInterface(
    rclcpp::Node *node, const std::string &interface_prefix)
    : node_{node} {
  param_ppr = node->create_publisher<std_msgs::msg::Int32>(
      interface_prefix + "/param/ppr", 10);

  param_ppr_get = node_->create_service<stepper_driver::srv::ParamPprGet>(
      interface_prefix + "/param/ppr/get",
      std::bind(&StepperDriverInterface::param_ppr_get_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
  param_ppr_set = node_->create_service<stepper_driver::srv::ParamPprSet>(
      interface_prefix + "/param/ppr/set",
      std::bind(&StepperDriverInterface::param_ppr_set_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
}

}  // namespace stepper_driver
