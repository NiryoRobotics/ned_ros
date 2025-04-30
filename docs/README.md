# Ned ROS stack Sphinx documentation

## Overview

This repository provides a Sphinx-based documentation tool for building and generating comprehensive documentation of the Ned ROS robotic stack using **ROS Noetic**. It includes custom directives to dynamically retrieve and document ROS-specific information such as topics, services, actions, messages... from a **running robotic stack in simulation**.

---

## Key features

* **Dynamic Content Parsing:** Automatically extracts data from a running simulated ROS stack.

* **Custom Sphinx Directives:**

    * `rosdoc`: General-purpose directive for ROS stacks.

    * `rosdoc_standalone_interface_package`: Specialized directive for packages containing only .msg, .srv, and .action files.

---

## Dependencies

- **Python 3.6+**
- **Python Packages:**
  - `Sphinx`
  - `sphinx_rtd_theme`
  - `sphinx-copybutton`
  - `sphinx-autobuild`
  - `sphinx-multiversion`
  - `jinja2`
  - `PyYAML`
  - `rospkg`
- **ROS Noetic**
- **ROS Packages:**
  - `ros-noetic-rosdoc-lite`

---

## Prerequisites

This documentation tool can only runs on the Ned ROS stack running **ROS Noetic** (Version 5.X.X of the stack).

### For local build 
1. Install [Docker](https://docs.docker.com/engine/install/ubuntu/) on your machine

2. Clone this repository
   ``` bash
   git clone ssh://git@gitlab01.niryotech.com:2224/documentations/ned-ros-stack-documentation.git
   ```
3. Get and start the ROS simulation stack docker container

   1. Pull the Ned ROS Noetic stack simulation docker image (Change the image with the one you want to use) and mount your local documentation repository in the ROS workspace of the container
   ``` bash
   sudo docker run --net=host -v .:/root/catkin_ws/src/docs/ gitlab01.niryotech.com:5050/robot/common/rpi-images/simulation_develop 
   ```

   2. If not already started, run you ROS simulation stack docker container and go to your documentation directory
   ``` bash
   sudo docker container start --interactive <container_name>
   ```

4. In a new terminal, access to the container using the `docker exec` command and go to your documentation directory
   ``` bash
   sudo docker exec --it <container_name> /bin/bash
   cd /root/catkin_ws/src/docs
   ```

5. Install Python dependencies
   1. Create a python virtual environement 
      ``` bash
      virtualenv venv
      ```
   2. Activate the virtual environment
      ``` bash
      source venv/bin/activate
      ```
   3. Install the required python packages with 
      ``` bash 
      pip install -r requirements.txt
      ```

6. Install [`rosdoc_lite`](http://wiki.ros.org/rosdoc_lite) which is ROS package used to autogenerate ROS documentation such as html files for interfaces
   ``` bash
   sudo apt update
   sudo apt-get install ros-noetic-rosdoc-lite
   ```

---


## How to build the documentation

1. In the docker container, run the robotic stack in simulation
   ``` bash
   roslaunch niryo_robot_bringup niryo_<robot_model>_simulation.launch
   ```
2. In another terminal, access to the container using the `docker exec` command
   ``` bash
   sudo docker exec -it <container_name> /bin/bash
   ```
3. Go to the `docs` folder, init the robot stack submodule if not done yet and build the documentation
   ``` bash
   cd catkin_ws/src/docs
   git submodule update --init
   sphinx-autobuild . .build/html
   ```
---

## Repository structure

```plaintext
docs/
├── .build/               # Output directory for built documentation
├── .static/              # Static files (CSS, JS, images, etc.)
├── .templates/           # Custom templates for Sphinx
├── rosdoc_extension/     # Custom Sphinx extension for ROS directives
|   ├── .build/           # Output directory for the rosdoc_lite built documentation
|   ├── templates/        # Jinja2 templates used by the custom directives to autogenerate .rst content             
|   ├── scripts/          # Helper scripts for the custom directives
├── source/               # Sphinx source files (index.rst, etc.)
├── Makefile              # Build commands for Sphinx
├── requirements.txt      # Python dependencies
├── index.rst             # Main page of the documentation
├── .gitignore            # .gitignore file
└── conf.py               # Sphinx configuration file
```

---

## Using Custom Directives

See [this documentation](./custom_directives.md) for more information on how to use the custom directives.

## Troubleshooting

* **Problem**: The ROS topics, services, or messages are not displayed in the documentation.

    **Solution**: Ensure the ROS stack is running before building the documentation. Check for errors in the terminal during the build process.

