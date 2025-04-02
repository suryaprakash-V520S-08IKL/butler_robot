Overview

The Butler Robot is a ROS-based package designed to handle order management and navigation in a restaurant environment. It listens for order confirmations, processes them, and navigates to the respective table to deliver the order.

Features

Order confirmation handling

Navigation to designated tables

ROS topic-based communication

Installation

Prerequisites

Ubuntu 20.04

ROS Noetic

Git

Clone the Repository

cd ~/catkin_ws/src
git clone https://github.com/suryaprakash-V520S-08IKL/butler_robot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash

Usage

Launch the Robot

roslaunch butler_robot butler_robot.launch

Publish Order Confirmation Message

rostopic pub /order_confirmation std_msgs/String "Table 3 confirmed"

File Structure

butler_robot/
│── CMakeLists.txt
│── package.xml
│── launch/
│   └── butler_robot.launch
│── msg/
│   └── Order.msg
│── scripts/
│   ├── confirmation_handler.py
│   ├── navigation.py
│   ├── order_manager.py

Contributors

Surya Prakash
