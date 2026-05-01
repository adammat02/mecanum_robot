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

    for (const hardware_interface::ComponentInfo &sensor : info_.sensors)
    {
      if (sensor.state_interfaces.empty())
      {
        RCLCPP_FATAL(get_logger(), "Sensor '%s' has no state interfaces.",
                     sensor.name.c_str());
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

    Wheel front_left_ = {cfg_.front_left_wheel_name, 0.0, 0.0, 0.0};
    Wheel front_right_ = {cfg_.front_right_wheel_name, 0.0, 0.0, 0.0};
    Wheel rear_left_ = {cfg_.rear_left_wheel_name, 0.0, 0.0, 0.0};
    Wheel rear_right_ = {cfg_.rear_right_wheel_name, 0.0, 0.0, 0.0};

    wheels_ = {front_left_, front_right_, rear_left_, rear_right_};

    cmd_rx_.rotations = {0.0, 0.0, 0.0, 0.0};
    data_valid_ = true;
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
    for (const auto &[name, descr] : sensor_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MecanumSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN:
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    try
    {
      serial_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms, '\r');
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to connect to serial device: %s", e.what());
      return hardware_interface::CallbackReturn::FAILURE;
    }
    if (!comm_.reset())
    {
      RCLCPP_ERROR(get_logger(), "Failed to reset STM over serial");
      return hardware_interface::CallbackReturn::FAILURE;
    }
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
    if (serial_.is_connected())
      serial_.disconnect();
    // END:

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type MecanumSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    // BEGIN:
    if (!serial_.is_connected())
      return hardware_interface::return_type::ERROR;

    double dt = period.seconds();

    if (!data_valid_)
    {
      RCLCPP_ERROR(get_logger(), "Failed to read wheel rotations over serial");
      return hardware_interface::return_type::ERROR;
    }

    // wheels
    for (size_t i = 0; i < wheels_.size(); ++i)
    {
      wheels_[i].pos_prev = wheels_[i].pos;
      wheels_[i].pos = cmd_rx_.rotations[i];
      wheels_[i].vel = (wheels_[i].pos - wheels_[i].pos_prev) / dt;
      set_state(wheels_[i].name + "/position", wheels_[i].pos * 2 * M_PI);
      set_state(wheels_[i].name + "/velocity", wheels_[i].vel * 2 * M_PI);
    }
    // battery
    set_state("battery_state/voltage", cmd_rx_.battery_voltage);

    data_valid_ = false;
    // END:
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type mecanum_robot ::MecanumSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN:
    if (!serial_.is_connected())
      return hardware_interface::return_type::ERROR;

    command_tx cmd_tx;
    for (size_t i = 0; i < wheels_.size(); ++i)
    {
      cmd_tx.speeds[i] = static_cast<double>(get_command(wheels_[i].name + "/velocity")) * 60.0 / (2 * M_PI);
    }

    const auto response = comm_.full_transaction(cmd_tx);
    if (!response)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send wheel speeds over serial");
      return hardware_interface::return_type::ERROR;
    }
    data_valid_ = true;
    cmd_rx_ = response.value();
    // END:

    return hardware_interface::return_type::OK;
  }

} // namespace mecanum_robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mecanum_robot::MecanumSystemHardware, hardware_interface::SystemInterface)
