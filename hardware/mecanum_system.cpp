// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mecanum_robot/mecanum_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mecanum_robot
{
  hardware_interface::CallbackReturn MecanumSystemHardware::on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params)
  {
    if (
        hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
    cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
    cfg_.rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];
    cfg_.rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];

    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

    cfg_.kp = hardware_interface::stod(info_.hardware_parameters["kp"]);
    cfg_.ki = hardware_interface::stod(info_.hardware_parameters["ki"]);
    cfg_.kd = hardware_interface::stod(info_.hardware_parameters["kd"]);


    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // MecanumSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MecanumSystemHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN:
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    front_left_ = {cfg_.front_left_wheel_name, 0.0, 0.0, 0.0};
    front_right_ = {cfg_.front_right_wheel_name, 0.0, 0.0, 0.0};
    rear_left_ = {cfg_.rear_left_wheel_name, 0.0, 0.0, 0.0};
    rear_right_ = {cfg_.rear_right_wheel_name, 0.0, 0.0, 0.0};

    // END:

    // reset values always when configuring hardware
    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      set_command(name, 0.0);
    }
    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MecanumSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN:
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    serial_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms, '\r');
    // END:

    // command and state should be equal when starting
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      set_command(name, get_state(name));
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MecanumSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN:
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
    serial_.disconnect();
    // END:

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type MecanumSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    // BEGIN:
    double dt = period.seconds();
    std::vector<double> rotations;
    front_left_.pos_prev = front_left_.pos;
    front_right_.pos_prev = front_right_.pos;
    rear_left_.pos_prev = rear_left_.pos;
    rear_right_.pos_prev = rear_right_.pos;

    if (!comm_.get_rotations(rotations))
    {
      RCLCPP_ERROR(get_logger(), "Failed to read wheel rotations over serial");
      return hardware_interface::return_type::ERROR;
    }

    front_left_.pos = rotations[0];
    front_right_.pos = rotations[1];
    rear_left_.pos = rotations[2];
    rear_right_.pos = rotations[3];

    front_left_.vel = (front_left_.pos - front_left_.pos_prev) / dt;
    front_right_.vel = (front_right_.pos - front_right_.pos_prev) / dt;
    rear_left_.vel = (rear_left_.pos - rear_left_.pos_prev) / dt;
    rear_right_.vel = (rear_right_.pos - rear_right_.pos_prev) / dt;

    set_state(front_left_.name + "/" + hardware_interface::HW_IF_POSITION, front_left_.pos * 2*M_PI);
    set_state(front_left_.name + "/" + hardware_interface::HW_IF_VELOCITY, front_left_.vel * 2*M_PI);

    set_state(front_right_.name + "/" + hardware_interface::HW_IF_POSITION, front_right_.pos * 2*M_PI);
    set_state(front_right_.name + "/" + hardware_interface::HW_IF_VELOCITY, front_right_.vel * 2*M_PI);

    set_state(rear_left_.name + "/" + hardware_interface::HW_IF_POSITION, rear_left_.pos * 2*M_PI);
    set_state(rear_left_.name + "/" + hardware_interface::HW_IF_VELOCITY, rear_left_.vel * 2*M_PI);

    set_state(rear_right_.name + "/" + hardware_interface::HW_IF_POSITION, rear_right_.pos * 2*M_PI);
    set_state(rear_right_.name + "/" + hardware_interface::HW_IF_VELOCITY, rear_right_.vel * 2*M_PI);
    // END:
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type mecanum_robot ::MecanumSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN:
    double vel1, vel2, vel3, vel4;
    vel1 = static_cast<double>(get_command(front_left_.name + "/" + hardware_interface::HW_IF_VELOCITY)) * 60.0 / (2*M_PI);
    vel2 = static_cast<double>(get_command(front_right_.name + "/" + hardware_interface::HW_IF_VELOCITY)) * 60.0 / (2*M_PI);
    vel3 = static_cast<double>(get_command(rear_left_.name + "/" + hardware_interface::HW_IF_VELOCITY)) * 60.0 / (2*M_PI);
    vel4 = static_cast<double>(get_command(rear_right_.name + "/" + hardware_interface::HW_IF_VELOCITY)) * 60.0 / (2*M_PI);

    if (!comm_.set_speeds({vel1, vel2, vel3, vel4}))
    {
      RCLCPP_ERROR(get_logger(), "Failed to send wheel speeds over serial");
      return hardware_interface::return_type::ERROR;
    }
    // END:

    return hardware_interface::return_type::OK;
  }

} // namespace mecanum_robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mecanum_robot::MecanumSystemHardware, hardware_interface::SystemInterface)
