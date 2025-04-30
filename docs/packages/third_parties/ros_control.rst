ROS control package
=========================================

`ros_control <http://wiki.ros.org/ros_control>`_ is a package in the ROS ecosystem designed for implementing and managing robot control systems. It provides a standardized and modular approach to controlling hardware interfaces, making it particularly suitable for robotic arms and similar systems.

Niryo robots rely heavily on this package to execute the following operations:

1. **Real-Time Capabilities**:
   The framework is built with real-time performance in mind, ensuring precise and responsive control for robotic actuators.

2. **Modular Architecture**:
   It uses controllers as modular components that can be dynamically loaded, swapped, or tuned at runtime.

3. **Support for Multiple Control Types**:
   Although the motors of Niryo robots are currently controlled only in position mode, ros_control supports various types of controllers, including:
   - Position controllers
   - Velocity controllers
   - Effort (torque/force) controllers

4. **Integration with ROS**:
   ros_control integrates seamlessly with ROS topics, services, and parameters, allowing for smooth communication between the control layer and other ROS nodes.

5. **Flexibility for Complex Systems**:
   It is ideal for multi-actuator systems like robotic arms, enabling coordinated motion.

