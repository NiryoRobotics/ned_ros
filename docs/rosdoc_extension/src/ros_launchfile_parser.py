import os
import yaml
from enum import Enum
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element
import rospkg
import rospy
from typing import List, Optional

from ros_info_models import ParameterInfo


class ConfigFileType(Enum):
    DEFAULT = "default"
    SIMULATION = "simulation"


class ROSLaunchParser:
    """
    A class to parse ROS launch files and extract simulatable parameters and dependant parameters.
    Simulatable parameters are parameters that may change if we are in simulation mode.
    A dependant parameter is a parameter that depends on a simulatable parameter.
    Therefore we can't add its dynamic value directly in the documentation, instead we add its raw value 
    """

    def __init__(self, simulation_param_keyword: str = "simulation_mode"):
        self._simulation_param_keyword = simulation_param_keyword
        self.rospack = rospkg.RosPack()

    def load_yaml_file(self, file_path: str) -> dict:
        """Loads a YAML file and returns its contents as a dictionary."""
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    def _check_and_update_parameter(self,
                                    parameters: List[ParameterInfo],
                                    target_name: str,
                                    new_values: Optional[dict] = None) -> bool:
        """
        Check if any object in the list has a name matching the target name. If found, update its fields with the provided values.

        :param parameters (List[ParameterInfo]): List of Parameter objects
        :param target_name (str): The name to search for
        :param new_values (Optional[dict]): A dictionary of fields to update with their new values
        :return: True if a match is found and updated, False otherwise
        """
        if new_values is None:
            return True
        
        for param in parameters:
            if param.name == target_name:
                for key, value in new_values.items():
                    if hasattr(param, key):
                        setattr(param, key, value)
                return True
        return False

    def _extract_simulation_arguments(
            self, launch_file_xml: Element) -> List[ParameterInfo]:
        """
        Extract arguments from a launch file that can have a simulated value.

        Args:
            launch_file_xml (Element): The root element of the launch file XML.

        Returns:
            List[ParameterInfo]: A list of arguments that are relevant to simulation.
        """
        simulation_arguments = []

        for arg in launch_file_xml.findall(".//arg"):
            arg_name = arg.get("name")
            value = arg.get("value")
            default = arg.get("default")
            arg_value = value if value is not None else default

            param = ParameterInfo(name=arg_name,
                                  simulation_value=None,
                                  default_value=None)

            if_instruction_found = 'if' in arg.attrib.keys()
            unless_instruction_found = 'unless' in arg.attrib.keys()
            sim_param_updated = self._check_and_update_parameter(simulation_arguments, arg_name, {"simulation_value": arg_value})
            real_param_updated = self._check_and_update_parameter(simulation_arguments, arg_name, {"default_value": arg_value})

            if if_instruction_found and self._simulation_param_keyword in arg.attrib.get("if"):
                param.simulation_value = arg_value
                if not sim_param_updated:
                    param.simulation_value = arg_value
                    simulation_arguments.append(param)
            elif unless_instruction_found and self._simulation_param_keyword in arg.attrib.get("unless"):
                if not real_param_updated:
                    param.default_value = arg_value
                    simulation_arguments.append(param)

        return simulation_arguments

    def _identify_yaml_file_type(self, yaml_file: str) -> str:
        """
        Identifies the type of a YAML file based on its name (Default or Simulation).

        Args:
            yaml_file (str): The path to the YAML file.

        Returns:
            str: The type of the YAML file.
        """
        if "simulation" in yaml_file:
            return ConfigFileType.SIMULATION
        else:
            return ConfigFileType.DEFAULT

    def _update_parameters_from_yaml(self, yaml_content: dict,
                                     yaml_file_type: ConfigFileType,
                                     parameters: List[ParameterInfo]) -> None:
        """
        Extract parameters from a YAML file and update a ParameterInfo list accordingly.

        Args:
            yaml_content (dict): The content of the YAML file.
            yaml_file_type (ConfigFileType): The type of the YAML file.
            parameters (List[ParameterInfo]): The list of parameters to update.
        """

        for param_name, param_value in yaml_content.items():

            new_param = ParameterInfo(name=param_name,
                                      simulation_value=None,
                                      default_value=None)

            if yaml_file_type == ConfigFileType.DEFAULT:
                if not self._check_and_update_parameter(
                        parameters, param_name,
                    {"default_value": param_value}):
                    new_param.default_value = param_value
                    parameters.append(new_param)

            elif yaml_file_type == ConfigFileType.SIMULATION:
                if not self._check_and_update_parameter(
                        parameters, param_name,
                    {"simulation_value": param_value}):
                    new_param.simulation_value = param_value
                    parameters.append(new_param)

        return parameters

    def _get_nested_path_key_values(self, d: dict) -> dict:
        """
        Recursively iterate through a dictionary and return a dictionary with the full nested path as keys.

        Args:
            d (dict): The input dictionary to be processed.

        Returns:
            dict: A dictionary containing the full nested path as keys.
        """
        end_key_values = {}

        def recurse(sub_dict, parent_key=""):
            for k, v in sub_dict.items():
                new_key = f"{parent_key}/{k}" if parent_key else k
                if isinstance(v, dict):
                    recurse(v, new_key)
                else:
                    end_key_values[new_key] = v

        recurse(d)
        return end_key_values

    def _extract_simulatable_parameters(
        self,
        launch_file_xml: Element,
        simulation_arguments: List[ParameterInfo],
    ) -> List[ParameterInfo]:
        """
        Args:
            launch_file_xml (Element): The XML element representing the launch file.
            simulation_arguments (List[ParameterInfo]): A list of simulation arguments to check against.

        Returns:
            List[ParameterInfo]: A list of simulatable parameters from the launch file that depend on the simulation arguments.
            
        """
        simulatable_parameters = []

        # Extract rosparam tag with yaml file from the launch file
        for rosparam in launch_file_xml.findall(
                ".//rosparam[@command='load']"):

            yaml_files = []
            yaml_file = rosparam.get("file")

            for simulation_argument in simulation_arguments:

                # Check if the simulation argument is present in the yaml file path,
                # if so, we need to have two yaml files (1 for simulation, and 1 default),
                # else return the default yaml file
                if simulation_argument.name in yaml_file:
                    default_yaml_file = yaml_file.replace(
                        f'$(arg {simulation_argument.name})',
                        simulation_argument.default_value)
                    simulation_yaml_file = yaml_file.replace(
                        f'$(arg {simulation_argument.name})',
                        simulation_argument.simulation_value)
                    yaml_files.append(default_yaml_file)
                    yaml_files.append(simulation_yaml_file)
                else:
                    yaml_files.append(yaml_file)

            for file in yaml_files:

                # if hardware version argument in file path, replace with the correct value
                hw_version_arg = "$(arg hardware_version)"
                if hw_version_arg in file:
                    hw_version = rospy.get_param(
                        "/niryo_robot/hardware_version")
                    file = file.replace(hw_version_arg, hw_version)

                file = self._resolve_ros_path(file)
                yaml_file_type = self._identify_yaml_file_type(file)
                yaml_content = self.load_yaml_file(file)

                # Parameters might be formatted differently between the yaml file and the loaded ROS params
                # Therefore, we retrieve the parameter key/value with the whole nested path of the key
                if yaml_content:
                    yaml_content = self._get_nested_path_key_values(yaml_content)

                self._update_parameters_from_yaml(yaml_content, yaml_file_type,
                                                  simulatable_parameters)

        # Extract param tag from the launch file
        for param in launch_file_xml.findall(".//param"):
            param_name = param.get("name")
            value = param.get("value")
            default = param.get("default")
            param_value = value if value is not None else default

            for simulation_argument in simulation_arguments:
                if simulation_argument.name in param_value:
                    default_value = param_value.replace(
                        f'$(arg {simulation_argument.name})',
                        simulation_argument.default_value)
                    simulation_value = param_value.replace(
                        f'$(arg {simulation_argument.name})',
                        simulation_argument.simulation_value)
                    param = ParameterInfo(name=param_name,
                                          default_value=default_value,
                                          simulation_value=simulation_value)
                    simulatable_parameters.append(param)

        return simulatable_parameters

    def _parse_simulatable_params_from_launch_file(
            self, file_path: str, simulation_arguments: List[ParameterInfo],
            simulatable_parameters: List[ParameterInfo]):
        """
        Recursively parse a ROS launch file to extract parameters and included launch files.

        :param file_path: Path to the launch file
        :param parameters: List to store the extracted parameters
        """
        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            return

        tree = ET.parse(file_path)
        root = tree.getroot()

        simulation_arguments.extend(self._extract_simulation_arguments(root))
        simulatable_parameters.extend(
            self._extract_simulatable_parameters(root, simulation_arguments))

        # Recursively parse included launch files
        for include in root.findall(".//include"):
            included_file = include.get("file")
            if included_file:
                resolved_file = self._resolve_ros_path(included_file)
                if os.path.exists(resolved_file):
                    self._parse_simulatable_params_from_launch_file(
                        resolved_file, simulation_arguments,
                        simulatable_parameters)

    def _resolve_ros_path(self, file_path):
        """Resolve ROS-style paths like $(find pkg) to absolute paths."""
        if "$(find " in file_path:
            pkg_name = file_path.split("$(find ")[1].split(")")[0]
            pkg_path = self._find_ros_package_path(pkg_name)
            if pkg_path:
                return file_path.replace(f"$(find {pkg_name})", pkg_path)
        return file_path

    def _find_ros_package_path(self, pkg_name):
        """Find the path of a ROS package using rospkg."""
        try:
            return self.rospack.get_path(pkg_name)
        except rospkg.ResourceNotFound:
            print(f"Package not found: {pkg_name}")
            return None

    def run(self, launch_file: str) -> List[ParameterInfo]:
        """
        Parses the given ROS launch file to extract simulatable and dependant parameters.

        Args:
            launch_file (str): The path to the ROS launch file to be parsed.

        Returns: 
            List[ParameterInfo]: Parameters that can be simulated.
        """
        simulatable_parameters: List[ParameterInfo] = []
        simulation_arguments: List[ParameterInfo] = []

        self._parse_simulatable_params_from_launch_file(
            launch_file, simulation_arguments, simulatable_parameters)

        return simulatable_parameters
