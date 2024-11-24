# Walker ROS2 Package for Obstacle Avoidance in Gazebo
[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD--3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Author 
Swaraj Mundruppady Rao
Email ID: swarajmr@umd.edu

## Overview

This repository contains a ROS2 package called **walker** that implements a simple walker algorithm for a robot in the Gazebo simulation environment. The robot mimics the behavior of a Roomba vacuum cleaner by moving forward until it detects an obstacle, rotating in place until the path ahead is clear, and alternating between clockwise and counterclockwise rotations. The project leverages a state machine design pattern for modularity and scalability.

## Features

- **State Machine Design**: Implements a robust state design pattern for easy state transitions.
- **Obstacle Detection**: Uses LIDAR data to detect obstacles in the forward direction.
- **Dynamic Rotation Direction**: Alternates between clockwise and counterclockwise rotations for obstacle avoidance.
- **Rosbag Integration**: Records all ROS2 topics except RGB-D camera topics to minimize file size.

## Repository Structure and Clone the Repository 
Download the release zip file and unzip the file and follow the following commands
1. Create a ros2 workspace
    ```bash
    mkdir -p ros2_ws/src/walker
    ```
2. cd into the walker directory
    ```bash
    cd ros2_ws/src/walker
    ```
3. Paste all the unzipped components inside the walker folder

The repository structure should be as follows 

```plaintext
ros2_ws/
├── src/
│   ├── walker/
│       ├── include/
│       │   ├── walker/
│       │       ├── walker_states.hpp
│       ├── src/
│       │   ├── walker_node.cpp
│       ├── launch/
│       │   ├── walker_launch.py
│       ├── Results/
│       │   ├── rosbag(rosbagfiles)
│       │   ├── cpplint_output.txt
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── README.md
│       ├── LICENSE
```

## Requirements or Dependencies

To get started with the **walker** ROS2 package, ensure you have the following installed:

- **ROS2 Humble**: Follow the [installation guide](https://docs.ros.org/en/humble/Installation.html).
- **Colcon Build Tool**: Install using the [official instructions](https://colcon.readthedocs.io/en/released/user/installation.html).
- **C++17 Compatible Compiler**: Ensure your compiler supports C++17 (e.g., GCC 7 or later).
- **Gazebo**: Install Gazebo by following the [installation guide](http://gazebosim.org/tutorials?tut=install_ubuntu).
- **TurtleBot3 Packages**: Install the TurtleBot3 packages using the [setup guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

Make sure all dependencies are properly installed before proceeding with the package setup.

## Build and Run the Setup
1. **Check for Missing Dependencies Before Building**

    To run rosdep in the root of the workspace: 

    ```bash
    cd ~/ros2_ws 
    rosdep install -i --from-path src --rosdistro humble -y
    ```
2. **Build the package**
    Use colcon to build the package 

    ```bash
    colcon build
    ```
3. **Source the setup**

    Source the script setup to overlay this workspace on the environment 
    ```bash
    source install/setup.bash
    ```
4. **Launch TurtleBot3 in a Simulation World**

    In a new terminal, run the following commands to set the TurtleBot3 model and launch it in the Gazebo simulation:

    ```bash
    echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models/" >> ~/.bashrc
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
    source ~/.bashrc
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```
5. **Launch the walker node with / without rosbag recording**
    ```bash
    ros2 launch walker walker_launch.py record_bag:=true
    ```
    Note : To run the walker node without rosbag recording use the following command 
    ```bash
    ros2 launch walker walker_launch.py record_bag:=false
    ```

6. Verifying the contents of the rosbag and Replaying the ROS Bag 
    To 
    verify the contents of the rosbag, use the following command:

    ```bash
    ros2 bag info <path_to_rosbag>
    ```

    To replay the rosbag, use:

    ```bash
    ros2 bag play <path_to_rosbag>
    ```


## Demo Video

Watch the demo video for this project:

[![Walker ROS2 Demo](https://img.youtube.com/vi/0AE1pd3-MHU/0.jpg)](https://youtu.be/0AE1pd3-MHU)

Click on the image above to view the video on YouTube.



