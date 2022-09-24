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

StepperDriverInterface::StepperDriverInterface(rclcpp::Node *node)
    : node_{node} {
  node->declare_parameter("stepper_driver_prefix", "/stepper_driver/default");
  node->get_parameter("stepper_driver_prefix", interface_prefix_);

  param_ppr = node->create_publisher<std_msgs::msg::Int32>(
      interface_prefix_.as_string() + "/param/ppr", 10);

  param_ppr_get = node_->create_service<stepper_driver::srv::ParamPprGet>(
      interface_prefix_.as_string() + "/param/ppr/get",
      std::bind(&StepperDriverInterface::param_ppr_get_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
  param_ppr_set = node_->create_service<stepper_driver::srv::ParamPprSet>(
      interface_prefix_.as_string() + "/param/ppr/set",
      std::bind(&StepperDriverInterface::param_ppr_set_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
}

}  // namespace stepper_driver
