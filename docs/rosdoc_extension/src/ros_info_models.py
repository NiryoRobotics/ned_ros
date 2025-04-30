from dataclasses import dataclass
from typing import List, Optional


@dataclass
class TopicInfo:
    name: str
    type: str
    description: str = "N/A"


@dataclass
class ServiceInfo:
    name: str
    type: str
    description: str = "N/A"


@dataclass
class ParameterInfo:
    name: str
    default_value: str
    simulation_value: Optional[str] = None
    unit: str = "N/A"
    description: str = "N/A"

@dataclass
class NodeInfo:
    publishers: List[TopicInfo]
    subscribers: List[TopicInfo]
    action_publishers: List[TopicInfo]
    action_subscribers: List[TopicInfo]
    services: List[ServiceInfo]
    parameters: List[ParameterInfo]
