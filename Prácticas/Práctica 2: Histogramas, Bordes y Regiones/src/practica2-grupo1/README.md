[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/ZX2VzbPH)
[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-2e0aaae1b6195c2367325f4f02e2d04e9abb55f0b24a779b69b11b9e10269abc.svg)](https://classroom.github.com/online_ide?assignment_repo_id=18456492&assignment_repo_type=AssignmentRepo)

# Computer Vision Examples

![distro](https://img.shields.io/badge/Ubuntu%2024-Nobley%20Numbat-green)
![distro](https://img.shields.io/badge/ROS2-Rolling-blue)
[![rolling](https://github.com/computer-vision-urjc/practica2-grupo1/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/computer-vision-urjc/practica2-grupo1/actions/workflows/rolling.yaml)

This project contains code examples created in Visual Studio Code for Computer Vision using C++ & OpenCV & Point Cloud Library (PCL) in ROS 2. These examples are created for the Computer Vision Subject of Robotics Software Engineering Degree at URJC.

# Installation 

You need to have previously installed ROS 2. Please follow this [guide](https://docs.ros.org/en/jazzy/Installation.html) if you don't have it.

```bash
source /opt/ros/jazzy/setup.bash
```

Clone the repository to your workspace:

```bash
mkdir -p ~/cv_ws/src
cd ~/cv_ws/src/
git clone https://<token>@github.com/jmguerreroh/computer_vision.git
git clone https://<token>@github.com/jmguerreroh/oak_d_camera.git
cd ~/cv_ws/
rosdep install --from-paths src --ignore-src -r -y
```

# Building project

```bash
colcon build --symlink-install
```

Once you have finished compiling the workspace with colcon, add the following lines to your bashrc file. The first makes a source of the workspace and the second configures the ROS 2 domain identifier to segment communication between nodes on the same network. This allows only nodes with the same ROS_DOMAIN_ID to interact, avoiding interference between different ROS 2 instances.

```bash
source ~/cv_ws/install/setup.bash
```

```bash
export ROS_DOMAIN_ID=1
```

# Run

To run this package, clone this repository into the src/ folder in your workspace:

```bash
cd ~/cv_ws/src/
```

```bash
git clone https://<token>@github.com/computer-vision-urjc/practica2-grupo1.git
```

Once the repository is cloned, compile the package in your workspace:

```bash
cd ~/cv_ws/
```

```bash
colcon build --packages-select practica2-grupo1
```

## **OPTION 1: WORK AT THE UNIVERSITY**

To run the practice from the university laboratories, all you have to do is launch the camera node in one terminal and the node used to carry out the practice in another terminal:

```bash
ros2 launch oak_d_camera rgbd_stereo.launch.py only_rgb:=true use_rviz:=false
```

```bash
ros2 launch practica2-grupo1 cv.launch.py
```

## **OPTION 2: WORK AT HOME**

Since we can't take the cameras home, we need to record a rosbag of the stage so we can have a preview of the stage saved.

First, a rosbag was recorded at the same time as the camera node was running, each of these commands executed in two different terminals:

```bash
ros2 bag record --all --output p2-g1-opX
```

```bash
ros2 launch oak_d_camera rgbd_stereo.launch.py ​​only_rgb:=true use_rviz:=false
```

Once this is done, stop the execution of both commands by pressing Ctrl+C.

And finally, play the recorded rosbag in one terminal while in a different terminal, run the practice node:

```bash
ros2 bag play --loop p2-g1-opX
```

```bash
ros2 launch practica2-grupo1 cv.launch.py
```

If you want to use your own robot, in the launcher, change the topic names to match the robot topics.

## FAQs:

* /usr/bin/ld shows libraries conflicts between two versions:

Probably you have installed and built your own OpenCV version, rename your local folder:

```bash
mv /usr/local/lib/cmake/opencv4 /usr/local/lib/cmake/oldopencv4
```

## About

This project was made by [José Miguel Guerrero], Associate Professor at [Universidad Rey Juan Carlos].

Copyright &copy; 2024.

[![Twitter](https://img.shields.io/badge/follow-@jm__guerrero-green.svg)](https://twitter.com/jm__guerrero)

## License

This work is licensed under the terms of the [MIT license](https://opensource.org/license/mit).

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[José Miguel Guerrero]: https://sites.google.com/view/jmguerrero
