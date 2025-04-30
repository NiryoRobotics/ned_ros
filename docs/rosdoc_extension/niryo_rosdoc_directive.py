import os
import yaml
from typing import Optional, List
from dataclasses import replace

from sphinx.application import Sphinx

from docutils import nodes

from src.abstract_rosdoc_directive import AbstractRosdocDirective
from src.ros_interface_parser import ROSInterfaceParser
from src.ros_launchfile_parser import ROSLaunchParser
from src.ros_info_models import NodeInfo, ParameterInfo

from sphinx.util import logging

logger = logging.getLogger(__name__)


class ROSDocDirective(AbstractRosdocDirective):
    required_arguments = 1
    optional_arguments = 0
    option_spec = {
        "action_namespace":
        str,  # Action namespace as a named option, used to filter action topic if they exist
        "description_file":
        str,  # path to the yaml file containing the descriptions of the ROS interfaces
        "patch_file":
        str,  # path to the yaml patch file used to make manual adjustments for documentation edge cases
        "is_global_namespace":
        bool,  # whether the ROS interfaces to analyze are bound to a global namespace or not (specific node)
        "can_be_simulated": lambda x:
        True,  # if a package can be simulated, change strategy to get parameters accordingly
        "launchfile_path":
        str,  # Path to the launch file to parse if can_be_simulated (relative to the current rst file path)
        "package_path_for_rosdoc_lite":
        str,  # Relative path to the package to build with rosdoc_lite
        "is_hardware_interface":
        lambda x: True,  # if the package is a hardware interface package
        "hardware_interface_node_namespace":
        str,  # The namespace used in the standalone hardware interface package if hardware_interface_node_namespace
    }
    has_content = False

    def run(self) -> List[nodes.Node]:

        if not self.arguments:
            return [
                self.state_machine.reporter.error(
                    'No ROS namespace provided for the rosdoc directive.',
                    line=self.lineno)
            ]

        if 'can_be_simulated' in self.options and 'launchfile_path' not in self.options:
            return [
                self.state_machine.reporter.error(
                    "The 'can_be_simulated' option requires the 'launchfile_path' option to be specified.",
                    line=self.lineno)
            ]

        if 'is_hardware_interface' in self.options and 'hardware_interface_node_namespace' not in self.options:
            return [
                self.state_machine.reporter.error(
                    "The 'is_hardware_interface' option requires the 'hardware_interface_node_namespace' option to be specified.",
                    line=self.lineno)
            ]

        ros_namespace = self.arguments[0]
        action_namespace = self.options.get("action_namespace", None)
        can_be_simulated = self.options.get("can_be_simulated", False)
        launchfile_path = self.options.get("launchfile_path", None)
        description_file_path = self.options.get("description_file", None)
        patch_file = self.options.get("patch_file", None)
        is_global_namespace = self.options.get("is_global_namespace", False)
        relative_package_path = self.options.get(
            "package_path_for_rosdoc_lite", None)
        hardware_interface_namespace = self.options.get(
            "hardware_interface_node_namespace", None)

        try:
            # Resolve the description file path
            if description_file_path:
                description_file_path = self.resolve_relative_path(
                    description_file_path)
            ros_parser = ROSInterfaceParser(description_file_path)

            filter_func = (lambda x: self.filter_by_namespace(
                x.name, hardware_interface_namespace)
                           ) if hardware_interface_namespace else None

            ros_info = ros_parser.parse_ros_stack(
                ros_namespace,
                is_global_namespace=is_global_namespace,
                action_namespace=action_namespace,
                filter_func=filter_func)

            # If the package can be simulated, parse the launch file to extract correct default values for parameters
            if can_be_simulated:
                launchfile_path = self.resolve_file_path(launchfile_path)

                launch_parser = ROSLaunchParser()
                simulatable_parameters = launch_parser.run(launchfile_path)

                # Update parameters in node's info
                self.update_node_parameters_with_simulatable_parameters(
                    ros_info.parameters, simulatable_parameters)

            # If a patch file is provided, update the node's info with the patch data
            if patch_file:
                patch_file_path = self.resolve_relative_path(patch_file)
                self.apply_patch(patch_file_path, ros_info)

            # Prepare data for template rendering
            data = self.prepare_data(ros_info, action_namespace)

            # Run rosdoc_lite on the package
            if relative_package_path:
                package_path = self.resolve_directory_path(
                    relative_package_path)
                self.run_rosdoc_lite(package_path)

            # Render the template
            template_dir = os.path.join(os.path.dirname(__file__), "templates")
            rendered_content = self.render_template(
                data,
                template_dir,
                "package_template.jinja",
                helper_functions={
                    "add_link_to_type_documentation":
                    self.add_link_to_type_documentation
                })

            # Parse and return the rendered content
            return self.parse_rst_content(rendered_content)

        except (FileNotFoundError, ValueError, KeyError) as e:
            logger.error(f"Error in ROSDocDirective: {e}")
            return [
                self.state_machine.reporter.warning(
                    f"Error in ROSDocDirective: {e}", line=self.lineno)
            ]

    def filter_by_namespace(self, item_name: str, namespace: str) -> bool:
        """
        Filter function to check if a namespace is part of an item name.

        Args:
            item_name (str): The name of the item (e.g., topic, service).
            namespace (str): The namespace to check for in the item name.

        Returns:
            bool: True if the namespace is in the item name, False otherwise.
        """
        return namespace in item_name

    def prepare_data(self, node_info: NodeInfo,
                     action_namespace: Optional[str]) -> dict:
        """
        Prepare the data dictionary to be passed to the Jinja2 template.

        Args:
            node_info (NodeInfo): An object containing information about the ROS node, 
                                  including publishers, subscribers, services, and parameters.
            action_namespace (Optional[str]): The namespace for ROS actions, if any.

        Returns:
            dict: A dictionary containing the following keys:
                - "topics": A dictionary with:
                    - "publishers": List of topics the node publishes to.
                    - "subscribers": List of topics the node subscribes to.
                - "action": A dictionary with:
                    - "namespace": The namespace for ROS actions.
                    - "publishers": List of action topics the node publishes to.
                    - "subscribers": List of action topics the node subscribes to.
                - "services": List of services provided by the node.
                - "parameters": List of parameters used by the node.
        """
        return {
            "topics": {
                "publishers": node_info.publishers,
                "subscribers": node_info.subscribers
            },
            "action": {
                "namespace": action_namespace,
                "publishers": node_info.action_publishers,
                "subscribers": node_info.action_subscribers
            },
            "services": node_info.services,
            "parameters": node_info.parameters
        }

    def apply_patch(self, patch_file: str, node_info: NodeInfo) -> None:
        """
        Apply a patch file to update the node's information.

        Args:
            patch_file (str): Path to the YAML patch file.
            node_info (NodeInfo): The node's information to be updated.
        """
        with open(patch_file, "r") as file:
            patch_data = yaml.safe_load(file)

        if "topics" in patch_data:
            node_info.publishers.extend(patch_data["topics"].get(
                "publishers", []))
            node_info.subscribers.extend(patch_data["topics"].get(
                "subscribers", []))
            node_info.action_publishers.extend(patch_data["topics"].get(
                "action_publishers", []))
            node_info.action_subscribers.extend(patch_data["topics"].get(
                "action_subscribers", []))

        if "services" in patch_data:
            node_info.services.extend(patch_data["services"])

        if "parameters" in patch_data:
            node_info.parameters.extend(patch_data["parameters"])

    def update_node_parameters_with_simulatable_parameters(
            self, parameters: List[ParameterInfo],
            new_parameters: List[ParameterInfo]) -> None:
        """
        Update the parameters field of NodeInfo objects with a list of SimulatableParameter objects.

        :param parameters: List of Parameter objects
        :param new_parameters: List of SimulatableParameter objects
        """
        # Make sure we do not update the same parameter multiple times
        updated_parameters = set()
        used_new_params = set()
        for index, parameter in enumerate(parameters):
            if parameter.name in updated_parameters:
                continue
            for new_param in new_parameters:
                if new_param.name in parameter.name and new_param.name not in used_new_params:
                    parameters[index] = replace(
                        parameter,
                        default_value=new_param.default_value,
                        simulation_value=new_param.simulation_value)
                    updated_parameters.add(parameter.name)
                    used_new_params.add(new_param.name)
                    break


def setup(app: Sphinx):
    app.add_directive("rosdoc", ROSDocDirective)

    return {
        'version': '0.1',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
