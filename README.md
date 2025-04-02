# Butler Robot

## Overview
The **Butler Robot** package is a ROS-based implementation designed to automate order management in a restaurant or service environment. It allows interaction with tables, processes orders, and navigates using predefined waypoints.

## Features
- Handles **order placement** and **order confirmation**.
- Navigates autonomously to designated tables.
- Integrates with ROS topics for real-time communication.
- Uses **custom ROS messages** for structured data exchange.

---

## Installation

### Prerequisites
Ensure you have **ROS Noetic** installed and a properly set up catkin workspace:
```bash
sudo apt update && sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

### Clone the Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/suryaprakash-V520S-08IKL/butler_robot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Package Structure
```
butler_robot/
│-- launch/
│   ├── butler_robot.launch      # Launch file
│-- msg/
│   ├── Order.msg                # Custom message definition
│-- scripts/
│   ├── confirmation_handler.py  # Handles order confirmation
│   ├── navigation.py            # Handles robot navigation
│   ├── order_manager.py         # Manages order processing
│-- CMakeLists.txt               # CMake build configuration
│-- package.xml                  # Package metadata
│-- README.md                    # Documentation
```

---

## Usage

### 1. Running the Butler Robot
Start the robot system using the launch file:
```bash
roslaunch butler_robot butler_robot.launch
```

### 2. Placing Orders
Orders are sent as messages on the `/order` topic.
```bash
"table_number: 3" 
"table_number: 2" 
"table_number: 1"
```

### 3. Confirming Orders
The confirmation handler listens to `/order_confirmation` and updates order status.
```bash
rostopic pub /order_confirmation std_msgs/String "data: 'Table 1 confirmed'"
rostopic pub /order_confirmation std_msgs/String "data: 'Table 2 confirmed'"
rostopic pub /order_confirmation std_msgs/String "data: 'Table 3 confirmed'"
```

### 4. Navigating to Tables
The robot navigates to a specific table after an order is placed.
```bash
rosrun butler_robot navigation.py
```

---

## ROS Topics
| Topic Name          | Message Type           | Description                    |
|---------------------|-----------------------|--------------------------------|
| `/order`           | `butler_robot/Order`  | Publishes new orders          |
| `/order_confirmation` | `std_msgs/String` | Confirms received orders      |
| `/cmd_vel`         | `geometry_msgs/Twist` | Controls robot movement       |

---

## Custom Messages

### Order.msg
```plaintext
int32 table_number
```

---

## Contributing
Feel free to submit **issues** or **pull requests** to enhance the package!

---


