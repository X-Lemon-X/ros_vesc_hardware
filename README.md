# VESC Hardware Interface

A ROS 2 hardware interface plugin for controlling VESC  motor controllers through the `ros2_control` framework.
Supporting multiple separated VESC controllers with individual configuration per motor.  
## Overview

This package provides a hardware interface implementation that bridges the gap between the `ros2_control` framework and VESC motor controllers. It enables seamless integration of VESC-based motor systems into ROS 2 robotics applications through a standardized control interface.

## Features

- **ros2_control Integration**: Implements the `hardware_interface` abstraction layer for full compatibility with the ros2_control ecosystem
- **CAN Communication**: Communicates with VESC controllers via CAN bus interface
- **Multiple Command Modes**: Supports various command interface types including velocity, position, current, and effort control
- **Dynamic Configuration**: Configurable parameters for different motor and application requirements
- **Hardware Lifecycle Management**: Proper initialization, activation, and deactivation through lifecycle management

## Dependencies
This package depends on the packages related to Nomad Project and ModuCard system:
- `can_device`
- `ari_shared_types`


### Configuration

The hardware interface is configured through the URDF/SDF description file and parameters:

- **can_interface**: The CAN device interface name (e.g., `can0`, `vcan0`)
- **command_interface_type**: Comma-separated list of command interface types per motor (e.g., `velocity,velocity`, `current,current`)
- **gear_ratios**: Comma-separated gear reduction ratios for each motor for accurate conversion between motor and output speeds
- **current_to_torque**: Comma-separated torque constants in mNm/A for each motor (conversion from current to torque)
- **polar_pairs**: Comma-separated number of pole pairs for each motor (used for calculating speed)
- **vesc_base_ids**: Comma-separated CAN base IDs for each VESC controller (must be > 0)
- **prefix**: Namespace prefix for joint names (optional, can be empty)

### Command Interfaces:
- **Velocity**: Direct velocity commands to the VESC motor controller
- **Position**: Position-based velocity commands with PID control
- **Current**: Direct current (torque) commands
- **Effort**: Effort-based control (simple good old fashion torque)

### State Interfaces:
- **Position**: Current position feedback from the motor
- **Velocity**: Current velocity feedback from the motor
- **Current**: Current feedback from the motor
- **Effort**: Effort feedback from the motor
- **Temperature**: Temperature feedback from the motor
- **Watt Hours**: Energy consumption feedback from the motor
- **Watt Hours Charged**: Energy charged feedback from the motor

## Usage

### Integration with ros2_control

Add the hardware interface plugin to your robot's URDF or via a separate config file:

like the one in `description/ros2_control/ros_control_vesc.xacro`

### Launching

Use the hardware interface through standard `ros2_control` controller managers and load your desired controllers (e.g., velocity controller, trajectory controller).

## Authors

- Patryk Dudziński (dodpat02@gmail.com)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
