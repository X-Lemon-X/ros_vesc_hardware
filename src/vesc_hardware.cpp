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

#include "can_device/can_device.hpp"
#include "can_device/can_messages.h"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "ari_shared_types/status.hpp"
#include "ari_shared_types/status_hardware.hpp"
#include "vesc_hardware/vesc_hardware.hpp"


namespace {
constexpr const char *nameHardwareInterface = "MCVescHardware";
} // namespace

using namespace ari;

template <typename T> Result<std::vector<T>> parse_string_array_to_vector(const std::string &input) {
  std::vector<T> result;
  size_t start = 0;
  size_t end   = input.find(',');

  try {
    while(end != std::string::npos) {
      if constexpr(std::is_same_v<T, std::string>) {
        result.push_back(input.substr(start, end - start));
      } else {
        result.push_back(static_cast<T>(std::stod(input.substr(start, end - start))));
      }
      start = end + 1;
      end   = input.find(',', start);
    }
    if constexpr(std::is_same_v<T, std::string>) {
      result.push_back(input.substr(start));
    } else {
      result.push_back(static_cast<T>(std::stod(input.substr(start))));
    }
  } catch(const std::exception &e) {
    return Status::ExpressionValidationError(std::string("Failed to parse string array to vector: ") + e.what());
  }
  return Result<std::vector<T>>::OK(std::move(result));
}

Status MCVescHardware::init(const hardware_interface::HardwareComponentInterfaceParams &iparams) {
  // (void)info;
  auto &info = iparams.hardware_info;
  prefix     = info.hardware_parameters.at("prefix");
  // can_interface_name is the name of the can interface we will use to communicate with the VESC
  auto can_interface_name = info.hardware_parameters.at("can_interface");
  // command_interface, whitch type of command interface we will use to control BLDC motors, velocity pos, curent, effort pos?
  auto maybe_command_interface_type = info.hardware_parameters.at("command_interface_type");
  // Gear ratio
  auto maybe_gear_ratio = info.hardware_parameters.at("gear_ratios");
  // mNm/A is the torque constant for the VESC motor
  auto maybe_current_to_torque = info.hardware_parameters.at("current_to_torque");
  // polar pairs is the number of pole pairs in the motor, used for calculating the speed
  auto maybe_polar_pairs = info.hardware_parameters.at("polar_pairs");

  auto maybe_vesc_base_ids = info.hardware_parameters.find("vesc_base_ids");

  ARI_ASIGN_OR_RETURN(array_command_interface_types,
                      parse_string_array_to_vector<std::string>(maybe_command_interface_type));
  ARI_ASIGN_OR_RETURN(array_gear_ratios, parse_string_array_to_vector<double>(maybe_gear_ratio));
  ARI_ASIGN_OR_RETURN(array_current_to_torque, parse_string_array_to_vector<double>(maybe_current_to_torque));
  ARI_ASIGN_OR_RETURN(array_polar_pairs, parse_string_array_to_vector<int>(maybe_polar_pairs));
  ARI_ASIGN_OR_RETURN(array_vesc_base_ids, parse_string_array_to_vector<int>(maybe_vesc_base_ids->second));

  auto joint_count = info.joints.size();
  if(array_command_interface_types.size() != joint_count) {
    return Status::ExpressionValidationError(
    "command_interface_type size does not match the number of joints");
  }

  if(array_gear_ratios.size() != joint_count) {
    return Status::ExpressionValidationError("gear_ratios size does not match the number of joints");
  }

  if(array_current_to_torque.size() != joint_count) {
    return Status::ExpressionValidationError("current_to_torque size does not match the number of joints");
  }

  if(array_polar_pairs.size() != joint_count) {
    return Status::ExpressionValidationError("polar_pairs size does not match the number of joints");
  }

  if(array_vesc_base_ids.size() != joint_count) {
    return Status::ExpressionValidationError("vesc_base_ids size does not match the number of joints");
  }

  joints.resize(joint_count);

  for(auto &&i : array_command_interface_types) {
    if(!(i == hardware_interface::HW_IF_VELOCITY || i == hardware_interface::HW_IF_CURRENT ||
         i == hardware_interface::HW_IF_EFFORT || i == hardware_interface::HW_IF_POSITION)) {
      return Status::ExpressionValidationError("Invalid command interface type: " + i);
    }
  }

  for(auto &&i : array_polar_pairs) {
    if(i <= 0) {
      return Status::ExpressionValidationError("Invalid polar pairs value: " + std::to_string(i) + ". Must be greater than 0.");
    }
  }

  for(auto &&i : array_vesc_base_ids) {
    if(i <= 0) {
      return Status::ExpressionValidationError("Invalid VESC base ID value: " + std::to_string(i) + ". Must be greater than 0");
    }
  }

  for(auto &&i : array_current_to_torque) {
    if(i <= 0) {
      return Status::ExpressionValidationError("Invalid Current to torque value: " + std::to_string(i) +
                                               ". Must be greater than 0.");
    }
  }

  for(const hardware_interface::ComponentInfo &joint : info.joints) {
    if(joint.command_interfaces.size() != 4) {
      return Status::ExpressionValidationError("Each joint must have exactly 4 command interfaces.");
    }

    if(!(joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
         joint.command_interfaces[0].name == hardware_interface::HW_IF_CURRENT ||
         joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT ||
         joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)) {
      return Status::ExpressionValidationError(
      "Invalid command interface type: " + joint.command_interfaces[0].name + ". Must be velocity, current, effort or position.");
    }
  }


  for(size_t i = 0; i < joint_count; i++) {
    integration_level_t command_interface;
    // Set command_interface based on command_interface_type
    auto command_interface_type = array_command_interface_types[i];
    if(command_interface_type == hardware_interface::HW_IF_VELOCITY) {
      command_interface = integration_level_t::VELOCITY;
    } else if(command_interface_type == hardware_interface::HW_IF_CURRENT) {
      command_interface = integration_level_t::CURRENT;
    } else if(command_interface_type == hardware_interface::HW_IF_EFFORT) {
      command_interface = integration_level_t::EFFORT;
    } else {
      command_interface = integration_level_t::VELOCITY; // default fallback
    }
    joints[i].command_interface = command_interface;
    joints[i].gear_ratio        = array_gear_ratios[i];
    joints[i].polar_pairs       = static_cast<double>(array_polar_pairs[i]);
    joints[i].current_to_torque = array_current_to_torque[i];
    joints[i].vesc_base_id      = array_vesc_base_ids[i];
    joints[i].name              = info.joints[i].name;
    RCLCPP_INFO(rclcpp::get_logger(nameHardwareInterface), "Joint %zu: name=%s command_interface_type=%s, gear_ratio=%.3f, polar_pairs=%d, current_to_torque=%.5f, vesc_base_id=%d",
                i, joints[i].name.c_str(), command_interface_type.c_str(), joints[i].gear_ratio,
                array_polar_pairs[i], joints[i].current_to_torque, joints[i].vesc_base_id);
  }

  ARI_ASIGN_TO_OR_RETURN(can_driver, CanDriver::Make(can_interface_name));


  RCLCPP_DEBUG(rclcpp::get_logger(nameHardwareInterface), "CAN driver created on interface %s",
               can_interface_name.c_str());


  return Status::OK();
}

hardware_interface::CallbackReturn
MCVescHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
  if(hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  auto status = init(params);
  if(!status.ok()) {
    RCLCPP_ERROR(rclcpp::get_logger(nameHardwareInterface), "Failed to initialize MCVescHardware: %s",
                 status.to_string().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MCVescHardware::on_configure(const rclcpp_lifecycle::State &previous_state) {
  (void)previous_state;

  for(const auto &[name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }
  for(const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }

  for(size_t i = 0; i < joints.size(); i++) {
    // thiss will generate callback frames for each of the motors
    uint32_t st1_id = (CAN_VESC_FLEFT_STATUS_1_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joints[i].vesc_base_id);
    uint32_t st2_id = (CAN_VESC_FLEFT_STATUS_2_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joints[i].vesc_base_id);
    uint32_t st3_id = (CAN_VESC_FLEFT_STATUS_3_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joints[i].vesc_base_id);
    uint32_t st4_id = (CAN_VESC_FLEFT_STATUS_4_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joints[i].vesc_base_id);
    uint32_t st5_id = (CAN_VESC_FLEFT_STATUS_5_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joints[i].vesc_base_id);
    uint32_t st6_id = (CAN_VESC_FLEFT_STATUS_6_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joints[i].vesc_base_id);

    ARI_HW_RETURN_ON_ERROR_NAM(can_driver->add_callback(st1_id,
                                                        std::bind(&MCVescHardware::can_callback_status_1, this,
                                                                  std::placeholders::_1, std::placeholders::_2,
                                                                  std::placeholders::_3),
                                                        &joints[i]),
                               nameHardwareInterface);

    ARI_HW_RETURN_ON_ERROR_NAM(can_driver->add_callback(st2_id,
                                                        std::bind(&MCVescHardware::can_callback_status_2, this,
                                                                  std::placeholders::_1, std::placeholders::_2,
                                                                  std::placeholders::_3),
                                                        &joints[i]),
                               nameHardwareInterface);

    ARI_HW_RETURN_ON_ERROR_NAM(can_driver->add_callback(st3_id,
                                                        std::bind(&MCVescHardware::can_callback_status_3, this,
                                                                  std::placeholders::_1, std::placeholders::_2,
                                                                  std::placeholders::_3),
                                                        &joints[i]),
                               nameHardwareInterface);

    ARI_HW_RETURN_ON_ERROR_NAM(can_driver->add_callback(st4_id,
                                                        std::bind(&MCVescHardware::can_callback_status_4, this,
                                                                  std::placeholders::_1, std::placeholders::_2,
                                                                  std::placeholders::_3),
                                                        &joints[i]),
                               nameHardwareInterface);

    ARI_HW_RETURN_ON_ERROR_NAM(can_driver->add_callback(st5_id,
                                                        std::bind(&MCVescHardware::can_callback_status_5, this,
                                                                  std::placeholders::_1, std::placeholders::_2,
                                                                  std::placeholders::_3),
                                                        &joints[i]),
                               nameHardwareInterface);

    ARI_HW_RETURN_ON_ERROR_NAM(can_driver->add_callback(st6_id,
                                                        std::bind(&MCVescHardware::can_callback_status_6, this,
                                                                  std::placeholders::_1, std::placeholders::_2,
                                                                  std::placeholders::_3),
                                                        &joints[i]),
                               nameHardwareInterface);
  }

  RCLCPP_INFO(rclcpp::get_logger(nameHardwareInterface), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MCVescHardware::on_activate(const rclcpp_lifecycle::State &previous_state) {
  (void)previous_state;
  ARI_HW_RETURN_ON_ERROR_NAM(can_driver->open_can(), nameHardwareInterface);

  for(size_t i = 0; i < joints.size(); i++) {
    joints[i].position           = 0.0;
    joints[i].velocity           = 0.0;
    joints[i].current            = 0.0;
    joints[i].erpm               = 0.0;
    joints[i].duty_cycle         = 0.0;
    joints[i].amd_hours          = 0.0;
    joints[i].amd_hours_charged  = 0.0;
    joints[i].watt_hours         = 0.0;
    joints[i].watt_hours_charged = 0.0;
    joints[i].temperature_mosfet = 0.0;
    joints[i].temperature_motor  = 0.0;
    joints[i].current_in         = 0.0;
    joints[i].pid_pos            = 0.0;
    joints[i].voltage            = 0.0;
    joints[i].adc1               = 0.0;
    joints[i].adc2               = 0.0;
    joints[i].adc3               = 0.0;
    joints[i].ppm                = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger(nameHardwareInterface), "System successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MCVescHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
  (void)previous_state;
  ARI_HW_RETURN_ON_ERROR_NAM(CanDriver::close_can(can_driver), nameHardwareInterface);

  RCLCPP_INFO(rclcpp::get_logger(nameHardwareInterface), "System successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MCVescHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
  // nothing to do here sinc vesc sends the data at random intervals
  (void)time;
  (void)period;
  for(auto &&i : joints) {
    set_state(prefix + i.name + "/position", i.position);
    set_state(prefix + i.name + "/velocity", i.velocity);
    set_state(prefix + i.name + "/current", i.current);
    set_state(prefix + i.name + "/effort", i.current * i.current_to_torque * i.gear_ratio);
    set_state(prefix + i.name + "/temperature", i.temperature_motor);
    set_state(prefix + i.name + "/watt_hours", i.watt_hours);
    set_state(prefix + i.name + "/watt_hours_charged", i.watt_hours_charged);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MCVescHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
  (void)time;
  (void)period;
  for(size_t i = 0; i < joints.size(); i++) {
    joint_t &joint = joints[i];
    CanFrame frame;
    frame.is_remote_request = false;

    switch(joint.command_interface) {
    case integration_level_t::CURRENT: {
      can_vesc_fleft_set_current_t str;
      // the value of current is in mA | so we need to multiply it by 1000
      double current = get_command<double>(prefix + joint.name + "/current") * 1000.0;
      str.current    = static_cast<int32_t>(current);
      frame.id = (CAN_VESC_FLEFT_SET_CURRENT_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joint.vesc_base_id);
      frame.size        = CAN_VESC_FLEFT_SET_CURRENT_LENGTH;
      frame.is_extended = CAN_VESC_FLEFT_SET_CURRENT_BRAKE_IS_EXTENDED;
      can_vesc_fleft_set_current_pack(frame.data, &str, frame.size);
      break;
    }
    case integration_level_t::VELOCITY: {
      can_vesc_fleft_set_rpm_t str;
      // for velocity we get rad/s so we need to convert it to rpm
      // 1 rad/s = 60 / (2 * PI) rpm
      double rpm = get_command<double>(prefix + joint.name + "/velocity") * 60.0 / (2.0 * M_PI) *
                   joint.gear_ratio * joint.polar_pairs;
      str.rpm    = static_cast<int32_t>(rpm);
      frame.id   = (CAN_VESC_FLEFT_SET_RPM_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joint.vesc_base_id);
      frame.size = CAN_VESC_FLEFT_SET_RPM_LENGTH;
      frame.is_extended = CAN_VESC_FLEFT_SET_RPM_IS_EXTENDED;
      can_vesc_fleft_set_rpm_pack(frame.data, &str, frame.size);
      break;
    }
    case integration_level_t::POSITION: {
      can_vesc_fleft_set_pos_t str;
      double pos = get_command<double>(prefix + joint.name + "/position") * 1000.0 *
                   joint.gear_ratio; // the value of position is in mDeg | so we need to multiply it by 1000
      str.position = static_cast<int32_t>(pos);
      frame.id   = (CAN_VESC_FLEFT_SET_POS_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joint.vesc_base_id);
      frame.size = CAN_VESC_FLEFT_SET_POS_LENGTH;
      frame.is_extended = CAN_VESC_FLEFT_SET_POS_IS_EXTENDED;
      can_vesc_fleft_set_pos_pack(frame.data, &str, frame.size);
      break;
    }
    case integration_level_t::EFFORT: {
      can_vesc_fleft_set_current_t str;
      double current =
      get_command<double>(prefix + joint.name + "/effort") / joint.current_to_torque * 1000.0 / joint.gear_ratio;
      str.current = static_cast<int32_t>(current);
      frame.id = (CAN_VESC_FLEFT_SET_CURRENT_FRAME_ID & 0xffffff00) | static_cast<uint32_t>(joint.vesc_base_id);
      frame.size        = CAN_VESC_FLEFT_SET_CURRENT_LENGTH;
      frame.is_extended = CAN_VESC_FLEFT_SET_CURRENT_IS_EXTENDED;
      can_vesc_fleft_set_current_pack(frame.data, &str, frame.size);
      break;
    }
    default:
      RCLCPP_ERROR(rclcpp::get_logger(nameHardwareInterface), "Unknown integration level %d",
                   static_cast<int>(joint.command_interface));
      return hardware_interface::return_type::ERROR;
    }
    // send the frame to the can bus
    can_driver->send(frame);
  }


  for(const auto &[name, descr] : sensor_state_interfaces_) {
    RCLCPP_INFO(rclcpp::get_logger(nameHardwareInterface), "Sensor: %s  Desc %s", name.c_str(),
                descr.interface_name.c_str());
  }

  return hardware_interface::return_type::OK;
}

void MCVescHardware::can_callback_status_1(CanDriver &can, const CanFrame &frame, void *args) {
  (void)can;
  joint_t *joint = static_cast<joint_t *>(args);

  can_vesc_fleft_status_1_t status;
  if(can_vesc_fleft_status_1_unpack(&status, frame.data, frame.size)) {
    RCLCPP_ERROR(rclcpp::get_logger(nameHardwareInterface), "Failed to unpack status 1");
    return;
  }
  joint->current    = static_cast<double>(status.current);
  joint->duty_cycle = static_cast<double>(status.duty);
  joint->erpm       = static_cast<double>(status.erpm) * joint->gear_ratio * joint->polar_pairs;
  joint->velocity   = static_cast<double>(status.erpm) / 60.0 * (2.0 * M_PI) /
                    (joint->gear_ratio * joint->polar_pairs); // convert rpm to rad/s

  // RCLCPP_INFO(rclcpp::get_logger(nameHardwareInterface), "ID: %i Current: %f  Duty: %f  Erpm: %f", frame.id,
  //             joint->current, joint->duty_cycle, joint->erpm);
}

void MCVescHardware::can_callback_status_2(CanDriver &can, const CanFrame &frame, void *args) {
  (void)can;
  joint_t *joint = static_cast<joint_t *>(args);
  can_vesc_fleft_status_2_t status;
  if(can_vesc_fleft_status_2_unpack(&status, frame.data, frame.size)) {
    RCLCPP_ERROR(rclcpp::get_logger(nameHardwareInterface), "Failed to unpack status 2");
    return;
  }
  joint->amd_hours         = static_cast<double>(status.amp_hours) * 1000.0;
  joint->amd_hours_charged = static_cast<double>(status.amp_hours_chg) * 1000.0;
}

void MCVescHardware::can_callback_status_3(CanDriver &can, const CanFrame &frame, void *args) {
  (void)can;
  joint_t *joint = static_cast<joint_t *>(args);
  can_vesc_fleft_status_3_t status;
  if(can_vesc_fleft_status_3_unpack(&status, frame.data, frame.size)) {
    RCLCPP_ERROR(rclcpp::get_logger(nameHardwareInterface), "Failed to unpack status 3");
    return;
  }
  joint->watt_hours         = static_cast<double>(status.wat_hours) * 1000.0;
  joint->watt_hours_charged = static_cast<double>(status.wat_hours_chg) * 1000.0;
}

void MCVescHardware::can_callback_status_4(CanDriver &can, const CanFrame &frame, void *args) {
  (void)can;
  joint_t *joint = static_cast<joint_t *>(args);
  can_vesc_fleft_status_4_t status;
  if(can_vesc_fleft_status_4_unpack(&status, frame.data, frame.size)) {
    RCLCPP_ERROR(rclcpp::get_logger(nameHardwareInterface), "Failed to unpack status 4");
    return;
  }
  joint->current_in         = static_cast<double>(status.current_in);
  joint->pid_pos            = static_cast<double>(status.pid_pos);
  joint->temperature_mosfet = static_cast<double>(status.temp_mosfet);
  joint->temperature_motor  = static_cast<double>(status.temp_motor);
}

void MCVescHardware::can_callback_status_5(CanDriver &can, const CanFrame &frame, void *args) {
  (void)can;
  joint_t *joint = static_cast<joint_t *>(args);
  can_vesc_fleft_status_5_t status;
  if(can_vesc_fleft_status_5_unpack(&status, frame.data, frame.size)) {
    RCLCPP_ERROR(rclcpp::get_logger(nameHardwareInterface), "Failed to unpack status 5");
    return;
  }
  //  This is weird tachometer that shows position of the wheel in degrees
  // but for some mysterious resone it shows that value in 180 deg per 360 deg rotation oven though
  // the control value input into the vesc are correct.
  static constexpr double scale_for_tachometer = 4.0 * M_PI / 360.0; //  2 * 2 * M_PI = 360 deg
  joint->position = static_cast<double>(status.tachometer) * scale_for_tachometer;
  joint->voltage  = static_cast<double>(status.volts_in) * 0.1;
  // RCLCPP_INFO(rclcpp::get_logger(nameHardwareInterface), "Pos: %f  Voltage: %f", joint->position, joint->voltage);
}

void MCVescHardware::can_callback_status_6(CanDriver &can, const CanFrame &frame, void *args) {
  (void)can;
  joint_t *joint = static_cast<joint_t *>(args);
  can_vesc_fleft_status_6_t status;
  if(can_vesc_fleft_status_6_unpack(&status, frame.data, frame.size)) {
    RCLCPP_ERROR(rclcpp::get_logger(nameHardwareInterface), "Failed to unpack status 6");
    return;
  }
  joint->adc1 = static_cast<double>(status.adc1) * 1000.0;
  joint->adc2 = static_cast<double>(status.adc2) * 1000.0;
  joint->adc3 = static_cast<double>(status.adc3) * 1000.0;
  joint->ppm  = static_cast<double>(status.ppm) * 1000.0;
}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MCVescHardware, hardware_interface::SystemInterface)