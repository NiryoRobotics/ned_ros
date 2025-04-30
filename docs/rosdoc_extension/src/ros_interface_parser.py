import re
import yaml
from enum import Enum
from typing import Optional, Dict, List, Tuple, Any, Callable

import rospy
from rosgraph.masterapi import Master
from rosservice import get_service_type, ROSServiceIOException
from rostopic import get_topic_type, ROSTopicIOException
from rosparam import list_params, RosParamIOException
import rosnode

from ros_info_models import TopicInfo, ServiceInfo, ParameterInfo, NodeInfo


class NodeInfoSection(Enum):
    PUBLICATIONS = "publications"
    SUBSCRIPTIONS = "subscriptions"
    SERVICES = "services"


class ROSInterfaceParser:

    def __init__(self, description_file_path: Optional[str] = None):
        # Load descriptions from the provided file path, if any
        self._descriptions = self._load_descriptions(description_file_path)

    def _load_descriptions(self,
                           path: Optional[str]) -> Optional[Dict[str, Any]]:
        if not path:
            return None
        with open(path, "r") as file:
            return yaml.safe_load(file)

    def _parse_global_infos(
        self,
        namespace: str,
        ros_master: Master,
        action_namespace: Optional[str] = None,
        descriptions: Optional[Dict[str, Any]] = None,
    ) -> Tuple[List[TopicInfo], List[TopicInfo], List[TopicInfo],
               List[TopicInfo], List[ServiceInfo]]:
        """
        Retrieve informations about global interfaces, including its topics and services, bound to a global namespace and not a specific node.

        Args:
            namespace (str): The namespace of the ROS node.
            master (Master): The ROS master object.
            action_namespace (Optional[str]): Optional namespace for actions.
            descriptions (Optional[Dict[str, Any]]): Optional dictionary containing descriptions of topics and services.

        Returns:
            Tuple[List[TopicInfo], List[TopicInfo], List[TopicInfo], List[TopicInfo], List[ServiceInfo]:
                A tuple containing lists of TopicInfo and ServiceInfo objects for publishers, subscribers,
                action publishers, action subscribers, and services.
        """
        publishers, subscribers, action_publishers, action_subscribers, services = [], [], [], [], []

        state = ros_master.getSystemState()
        pubs = [topic for topic, _ in state[0] if topic.startswith(namespace)]
        subs = [topic for topic, _ in state[1] if topic.startswith(namespace)]
        srvs = [
            service for service, _ in state[2] if service.startswith(namespace)
        ]

        for topic in pubs:
            topic_info = self._get_topic_info(topic, descriptions)
            if action_namespace and action_namespace in topic:
                action_publishers.append(topic_info)
            else:
                publishers.append(topic_info)

        for topic in subs:
            topic_info = self._get_topic_info(topic, descriptions)
            if action_namespace and action_namespace in topic:
                action_subscribers.append(topic_info)
            else:
                subscribers.append(topic_info)

        for service in srvs:
            service_info = self._get_service_info(service, descriptions)
            services.append(service_info)

        return publishers, subscribers, action_publishers, action_subscribers, services

    def _parse_node_infos(
        self,
        node_name: str,
        action_namespace: Optional[str] = None,
        descriptions: Optional[Dict[str, Any]] = None,
        filter_func: Optional[Callable[
            [str], bool]] = None,  # External filter function
    ) -> Tuple[List[TopicInfo], List[TopicInfo], List[TopicInfo],
               List[TopicInfo], List[ServiceInfo]]:
        """
        Retrieve information about a ROS node, including its topics and services.

        Args:
            node_name (str): The name of the node.
            action_namespace (Optional[str]): Optional namespace for actions.
            descriptions (Optional[Dict[str, Any]]): Optional dictionary containing descriptions of topics and services.
            filter_func (Optional[Callable[[str], bool]]): Optional function to filter topics and services.

        Returns:
            Tuple[List[TopicInfo], List[TopicInfo], List[TopicInfo], List[TopicInfo], List[ServiceInfo]]:
                A tuple containing lists of TopicInfo and ServiceInfo objects for publishers, subscribers,
                action publishers, action subscribers, and services.
        """
        node_info = rosnode.get_node_info_description(node_name).splitlines()
        publishers, subscribers, action_publishers, action_subscribers, services = [], [], [], [], []
        current_section = None

        for line in node_info:
            line = line.strip()

            # Identify section headers
            current_section = self._identify_section(line, current_section)

            # Parse entries based on the current section
            if current_section == NodeInfoSection.PUBLICATIONS:
                self._parse_topic_line(line, publishers, action_publishers,
                                       action_namespace, descriptions)
            elif current_section == NodeInfoSection.SUBSCRIPTIONS:
                self._parse_topic_line(line, subscribers, action_subscribers,
                                       action_namespace, descriptions)
            elif current_section == NodeInfoSection.SERVICES:
                self._parse_service_line(line, services, descriptions)

        if filter_func:
            publishers = list(filter(filter_func, publishers))
            subscribers = list(filter(filter_func, subscribers))
            action_publishers = list(filter(filter_func, action_publishers))
            action_subscribers = list(filter(filter_func, action_subscribers))
            services = list(filter(filter_func, services))

        return publishers, subscribers, action_publishers, action_subscribers, services

    def _identify_section(self, line: str,
                          current_section: Optional[str]) -> Optional[str]:
        """
        Identify the section of the node information based on the line content.

        Args:
            line (str): The current line being processed.
            current_section (Optional[str]): The current section being processed.

        Returns:
            Optional[str]: The identified section or None if the line does not match any section.
        """
        if line.startswith("Publications:"):
            current_section = NodeInfoSection.PUBLICATIONS
        elif line.startswith("Subscriptions:"):
            current_section = NodeInfoSection.SUBSCRIPTIONS
        elif line.startswith("Services:"):
            current_section = NodeInfoSection.SERVICES
        elif not line:
            current_section = None
        return current_section

    def _parse_topic_line(
        self,
        line: str,
        topic_list: List[TopicInfo],
        action_list: List[TopicInfo],
        action_namespace: Optional[str] = None,
        descriptions: Optional[Dict[str, Any]] = None,
    ) -> None:
        match = re.match(r"\* ([^ ]+) \[([^\]]+)\]", line)
        if match:
            topic_name = match.group(1)
            topic_info = self._get_topic_info(topic_name, descriptions)
            if action_namespace and action_namespace in topic_name:
                action_list.append(topic_info)
            else:
                topic_list.append(topic_info)

    def _parse_service_line(
        self,
        line: str,
        service_list: List[ServiceInfo],
        descriptions: Optional[Dict[str, Any]] = None,
    ) -> None:
        if line.startswith("* "):
            service_name = line[2:]  # Remove "* " prefix
            service_info = self._get_service_info(service_name, descriptions)
            service_list.append(service_info)

    def _get_topic_info(
            self,
            topic_name: str,
            descriptions: Optional[Dict[str, Any]] = None) -> TopicInfo:
        """
        Retrieve topic type and description.

        Args:
            topic_name (str): The name of the topic.
            descriptions (Optional[Dict[str, Any]]): Optional dictionary containing descriptions of topics.

        Returns:
            TopicInfo: An object containing the topic name, type, and description.
        """
        try:
            topic_type = get_topic_type(topic_name)[0]
        except ROSTopicIOException:
            topic_type = "unknown"

        description = descriptions.get("topics", {}).get(topic_name, "N/A")

        return TopicInfo(name=topic_name,
                         type=topic_type,
                         description=description)

    def _get_service_info(
            self,
            service_name: str,
            descriptions: Optional[Dict[str, Any]] = None) -> ServiceInfo:
        """
        Retrieve service type and description.

        Args:
            service_name (str): The name of the service.
            descriptions (Optional[Dict[str, Any]]): Optional dictionary containing descriptions of services.

        Returns:
            ServiceInfo: An object containing the service name, type, and description.
        """
        try:
            service_type = get_service_type(service_name)
        except ROSServiceIOException:
            service_type = "unknown"

        description = descriptions.get("services", {}).get(service_name, "N/A")

        return ServiceInfo(name=service_name,
                           type=service_type,
                           description=description)

    def _parse_parameters(
            self,
            namespace: str,
            filter_func: Optional[Callable[[str], bool]] = None,
            descriptions: Optional[Dict[str,
                                        Any]] = None) -> List[ParameterInfo]:
        """
        Parse parameters for a given namespace, including their default values.

        Args:
            namespace (str): The namespace to parse parameters from.
            filter_func (Optional[Callable[[str], bool]]): Optional function to filter parameters.
            descriptions (Optional[Dict[str, Any]]): Optional dictionary containing descriptions of parameters.

        Returns:
            List[ParameterInfo]: A list of ParameterInfo objects containing parameter details.
        """
        parameters = list_params(namespace)

        parameters_info = [
            self._get_parameter_info(param, descriptions)
            for param in parameters
        ]

        if filter_func:
            parameters_info = list(filter(filter_func, parameters_info))

        return parameters_info

    def _get_parameter_info(
            self,
            parameter_name: str,
            descriptions: Optional[Dict[str, Any]] = None) -> ParameterInfo:
        """
        Retrieve information about a ROS parameter, including its default value and description.

        Args:
            parameter_name (str): The name of the ROS parameter to retrieve information for.
            descriptions (Optional[Dict[str, Any]]): An optional dictionary containing parameter descriptions.

        Returns:
            ParameterInfo: An object containing the parameter's name, default value, and description.
        """
        try:
            default_value = rospy.get_param(parameter_name)
        except RosParamIOException:
            default_value = "unknown"

        param_data = descriptions.get("parameters", {}).get(parameter_name, {})

        description = param_data.get("text", "N/A")
        unit = param_data.get("unit", "N/A")
        return ParameterInfo(name=parameter_name,
                             default_value=default_value,
                             unit=unit,
                             description=description)

    def parse_ros_stack(self,
                        namespace: str,
                        filter_func: Optional[Callable[[str], bool]] = None,
                        is_global_namespace: bool = False,
                        action_namespace: Optional[str] = None) -> NodeInfo:
        """
        Parse ROS stack information, including topics, services, and parameters.

        Args:
            namespace (str): The name of the ROS namespace to parse.
            filter_func (Optional[Callable[[str], bool]]): Optional function to filter topics, services and parameters.
            is_global_namespace (bool): Flag indicating if the namespace is global or not (bound to a specific node).
            action_namespace (Optional[str]): Optional namespace for actions.

        Returns:
            NodeInfo: An object containing information about the node's publishers, subscribers,
                      action publishers, action subscribers, services, and parameters.
        """

        master = Master('/ros_master')
        if not master.is_online():
            raise Exception(f"Unable to communicate with master!")

        rospy.init_node("ros_interface_parser", anonymous=True)

        if is_global_namespace:
            pubs_info, subs_info, action_pubs_info, action_subs_info, services_info = self._parse_global_infos(
                namespace,
                master,
                action_namespace=action_namespace,
                descriptions=self._descriptions)
        else:
            pubs_info, subs_info, action_pubs_info, action_subs_info, services_info = self._parse_node_infos(
                namespace,
                filter_func=filter_func,
                action_namespace=action_namespace,
                descriptions=self._descriptions)

        parameters_info = self._parse_parameters(
            namespace,
            filter_func=filter_func,
            descriptions=self._descriptions)

        return NodeInfo(publishers=pubs_info,
                        subscribers=subs_info,
                        action_publishers=action_pubs_info,
                        action_subscribers=action_subs_info,
                        services=services_info,
                        parameters=parameters_info)
