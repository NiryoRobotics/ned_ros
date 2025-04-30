## Using Custom Directives

### **1. ****`rosdoc`**** Directive**

This directive generates documentation for a running ROS node, including topics, services, actions, and parameters.
You can edit the generated content by providing a YAML file containing interface descriptions (in `docs/packages/descriptions.yaml`) and a YAML patch file for manual adjustments (examples in `docs/packages/rosdoc_patches/`).
You can also edit the format of the generated content by providing a Jinja2 template file (in `docs/rosdoc_extension/templates/`).

#### **Directive Argument**

| Argument        | Description                                           |
| --------------- | ----------------------------------------------------- |
| `ros_namespace` | Namespace of the ROS node which needs to be inspected |

#### **Directive Options**

| Option                             | Type   | Description                                                                                         |
| -----------------------------------|--------| ----------------------------------------------------------------------------------------------------|
| `action_namespace`                 | string | Namespace of the action for filtering action topics.                                                |
| `description_file`                 | string | Path to a YAML file containing interface descriptions. Relative to the Sphinx docs root.            |
| `patch_file`                       | sring  | Path to a YAML patch file for manual adjustments. Relative to the Sphinx docs root.                 |
| `is_global_namespace`              | bool   | Whether to analyze interfaces in the global namespace (if the interfaces are not bound to a specific node).                                                                                                                                              |
| `can_be_simulated`                 | None   | Indicates if the package can be simulated. A package can be simulated if its launchfile calls different parameters whether we are in simulation mode or not. Requires `launchfile_path`.                                                                               |
| `launchfile_path`                  | string | Path to the launch file which needs to be analyzed (required if `can_be_simulated` is set). Relative to the current .rst file.                                                                                                                                               |
| `package_path_for_rosdoc_lite`     | string | Path to the ROS package for `rosdoc_lite` build. Relative to the current .rst file.                 |
| `is_hardware_interface`            | None   | Indicates if the package is a hardware interface. Requires a `hardware_interface_node_namespace` which inspects the interfaces exposed by the node which contains the `hardware_interface_node_namespace`.                                                              |
| `hardware_interface_node_namespace`| string | Namespace for hardware interface packages.                                                          |

#### **Example Usage**

##### **Standard Usage**
```rst
.. rosdoc:: /niryo_robot_led_ring
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../../../niryo_robot_led_ring
```

##### **The node implements an action server**
```rst
.. rosdoc:: /niryo_robot_arm_commander 
    :action_namespace: robot_action
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../../../niryo_robot_arm_commander
```
##### **The generated documentation should be manually patched**
```rst
.. rosdoc:: /niryo_robot_vision
    :description_file: packages/descriptions.yaml
    :patch_file: packages/rosdoc_patches/niryo_robot_vision_patch.yaml
    :package_path_for_rosdoc_lite: ../../../niryo_robot_vision
    :can_be_simulated:
    :launchfile_path: ../../../niryo_robot_vision/launch/vision_node.launch
```

##### **The node can be simulated**
```rst
.. rosdoc:: /niryo_robot_poses_handlers
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../../../niryo_robot_poses_handlers
    :can_be_simulated:
    :launchfile_path: ../../../niryo_robot_poses_handlers/launch/poses_handlers.launch
```

##### **The node is a hardware interface node**
```rst
.. rosdoc:: /niryo_robot_hardware_interface
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../../../niryo_robot_hardware_stack/joints_interface
    :can_be_simulated:
    :launchfile_path: ../../../niryo_robot_hardware_stack/joints_interface/launch/joints_interface_base.launch.xml
    :is_hardware_interface:
    :hardware_interface_node_namespace: /joints_interface
```

##### **The namespace is not bound to a specific node**
```rst
.. rosdoc:: /niryo_robot/
    :is_global_namespace: True
    :description_file: packages/descriptions.yaml
```

### **2. ****`rosdoc_standalone_interface_package`**** Directive**

This directive targets ROS packages that only contain `.msg`, `.srv`, and `.action` files.
It will generate documentation for the package's interfaces using `rosdoc_lite`.

#### **Directive Argument**

| Argument       | Description                                          |
| -------------- | ---------------------------------------------------- |
| `package_path` | Path to the ROS package relative to the `.rst` file. |

#### **Example Usage**

```rst
.. rosdoc_standalone_interface_package:: ../../niryo_robot_msgs
```

---