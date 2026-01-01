/*
Copyright (c) 2025 Patryk Dudziński

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * Authors: Patryk Dudziński
 */


#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "can_device/can_device.hpp"
#include "vesc_hardware/visiblity_control.h"

namespace nomad_hardware {

class MCVescHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MCVescHardware)

  HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;


  Status init(const hardware_interface::HardwareInfo &info);

  HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void can_callback_status_1(CanDriver &can, const CanFrame &frame, void *args);
  void can_callback_status_2(CanDriver &can, const CanFrame &frame, void *args);
  void can_callback_status_3(CanDriver &can, const CanFrame &frame, void *args);
  void can_callback_status_4(CanDriver &can, const CanFrame &frame, void *args);
  void can_callback_status_5(CanDriver &can, const CanFrame &frame, void *args);
  void can_callback_status_6(CanDriver &can, const CanFrame &frame, void *args);

  enum integration_level_t : std::uint8_t { POSITION = 1, VELOCITY = 2, CURRENT = 3, EFFORT = 4 };


  struct joint_t {
    // Parameters
    double gear_ratio;
    double polar_pairs;
    double current_to_torque;
    int vesc_base_id;
    std::string name;

    // States
    double position;
    double velocity;
    double current;

    double erpm;
    double duty_cycle;
    double amd_hours;
    double amd_hours_charged;
    double watt_hours;
    double watt_hours_charged;
    double temperature_mosfet;
    double temperature_motor;
    double current_in;
    double pid_pos;
    double voltage;

    double adc1;
    double adc2;
    double adc3;
    double ppm;
    integration_level_t command_interface;
  };

  std::vector<joint_t> joints;
  std::string prefix;
  std::shared_ptr<CanDriver> can_driver;
};

} // namespace nomad_hardware
