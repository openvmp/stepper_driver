/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_STEPPER_DRIVER_INTERFACE_H
#define OPENVMP_STEPPER_DRIVER_INTERFACE_H

#include <memory>
#include <string>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/implementation.hpp"
#include "remote_actuator/srv/velocity_set.hpp"
#include "std_msgs/msg/int32.hpp"
#include "stepper_driver/srv/param_ppr_get.hpp"
#include "stepper_driver/srv/param_ppr_set.hpp"

namespace stepper_driver {

class Node;

class Interface : public remote_actuator::Implementation {
 public:
  Interface(rclcpp::Node *node);
  virtual ~Interface() {}

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr param_ppr;

  rclcpp::Service<stepper_driver::srv::ParamPprGet>::SharedPtr param_ppr_get;
  rclcpp::Service<stepper_driver::srv::ParamPprSet>::SharedPtr param_ppr_set;

 protected:
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Parameter interface_prefix_;

  std::string get_prefix_();

  // pass 'velocity_set_real_' through
  virtual void velocity_set_real_(double velocity) = 0;

  virtual rclcpp::FutureReturnCode param_ppr_get_handler_(
      const std::shared_ptr<stepper_driver::srv::ParamPprGet::Request> request,
      std::shared_ptr<stepper_driver::srv::ParamPprGet::Response> response) = 0;
  virtual rclcpp::FutureReturnCode param_ppr_set_handler_(
      const std::shared_ptr<stepper_driver::srv::ParamPprSet::Request> request,
      std::shared_ptr<stepper_driver::srv::ParamPprSet::Response> response) = 0;
};

}  // namespace stepper_driver

#endif  // OPENVMP_STEPPER_DRIVER_INTERFACE_H
