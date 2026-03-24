// Copyright 2024 Husarion sp. z o.o.
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

#include "agv_pro_hardware_interfaces/agv_pro_imu_sensor.hpp"

#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

namespace agv_pro_hardware_interfaces
{
hardware_interface::CallbackReturn
AgvProImuSensor::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(rclcpp::get_logger("AgvProImuSensor"), "Initializing IMU sensor");

  if (hardware_interface::SensorInterface::on_init(hardware_info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AgvProImuSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Linear acceleration
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.x", &linear_acceleration_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.y", &linear_acceleration_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.z", &linear_acceleration_z_));

  // Angular velocity
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.x", &angular_velocity_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.y", &angular_velocity_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.z", &angular_velocity_z_));

  // Orientation
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.x", &orientation_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.y", &orientation_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.z", &orientation_z_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.w", &orientation_w_));

  return state_interfaces;
}

hardware_interface::CallbackReturn
AgvProImuSensor::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AgvProImuSensor"), "Activating IMU sensor");

  // Initialize IMU data
  linear_acceleration_x_ = 0.0;
  linear_acceleration_y_ = 0.0;
  linear_acceleration_z_ = 9.81;  // Gravity
  
  angular_velocity_x_ = 0.0;
  angular_velocity_y_ = 0.0;
  angular_velocity_z_ = 0.0;
  
  orientation_x_ = 0.0;
  orientation_y_ = 0.0;
  orientation_z_ = 0.0;
  orientation_w_ = 1.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AgvProImuSensor::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AgvProImuSensor"), "Deactivating IMU sensor");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
AgvProImuSensor::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Read actual IMU data from AGV Pro
  // For now, we'll just return simulated data
  RCLCPP_DEBUG(rclcpp::get_logger("AgvProImuSensor"), "Reading IMU data");

  // Simulate small noise
  linear_acceleration_x_ += 0.01 * (rand() % 3 - 1);
  linear_acceleration_y_ += 0.01 * (rand() % 3 - 1);
  linear_acceleration_z_ = 9.81 + 0.01 * (rand() % 3 - 1);
  
  angular_velocity_x_ += 0.005 * (rand() % 3 - 1);
  angular_velocity_y_ += 0.005 * (rand() % 3 - 1);
  angular_velocity_z_ += 0.005 * (rand() % 3 - 1);

  return hardware_interface::return_type::OK;
}

}  // namespace agv_pro_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  agv_pro_hardware_interfaces::AgvProImuSensor,
  hardware_interface::SensorInterface)
