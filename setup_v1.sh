#!/bin/bash
# script to set all the configuration files for NED V1 as default

cp ./niryo_robot_tools_commander/config/end_effectors_v1.yaml ./niryo_robot_toolsi_commander/config/end_effectors.yaml
cp ./niryo_robot_hardware_stack/joints_interface/config/ned/dynamixels_params_v1.yaml ./niryo_robot_hardware_stack/joints_interface/config/ned/dynamixels_params.yaml
cp ./niryo_robot_hardware_stack/joints_interface/config/joints_params_v1.yaml ./niryo_robot_hardware_stack/joints_interface/config/joints_params.yaml
cp ./niryo_robot_hardware_stack/joints_interface/config/motors_param_v1.yaml ./niryo_robot_hardware_stack/joints_interface/config/motors_param.yaml
cp ./niryo_robot_hardware_stack/ttl_driver/config/motors_config_v1.yaml ./niryo_robot_hardware_stack/ttl_driver/config/motors_config.yaml
cp ./niryo_robot_hardware_stack/tools_interface/config/default_v1.yaml ./niryo_robot_hardware_stack/tools_interface/config/default.yaml

