# My Robot Controller ROS Package

This ROS package, named my_robot_controller, consists of several nodes for controlling a robot in a simulated environment using ROS 2 (Robot Operating System 2)

## Nodes

### 1. my_first_node

- **File:** `my_first_node.py`
- **Description:** This node demonstrates a basic ROS node setup. It initializes a timer that periodically logs a message  ndicating the number of times the timer callback is executed.
- **Usage:** Run this node to observe periodic log messages.

### 2. draw_circle

- **File:** `draw_circle.py`
- **Description:** This node controls a simulated robot to draw a circle by publishing Twist messages to the /turtle1/cmd_vel topic.
- **Usage:** Run this node to make the robot draw a circle in the simulated environment.

### 3. pose_subscriber

- **File:** `pose_subscriber.py`
- **Description:** This node subscribes to the pose messages published by the turtle in the simulated environment and logs the received pose information.
- **Usage:** Run this node to observe the pose of the turtle robot.

### 4. turtle_controller

- **File:** `turtle_controller.py`
- **Description:** This node controls the turtle robot's movements and pen color based on its pose. It subscribes to the pose messages, calculates the appropriate velocity commands, and calls the SetPen service to change the pen color.
- **Usage:** Run this node to control the turtle robot's movements and pen color.

## File Structure


``bash
my_robot_controller
    ├── my_robot_controller
    │   ├── draw_circle.py
    │   ├── __init__.py
    │   ├── my_first_node.py
    │   ├── pose_subscriber.py
    │   ├── __pycache__
    │   │   ├── __init__.cpython-310.pyc
    │   │   └── my_first_node.cpython-310.pyc
    │   └── turtle_controller.py

``

## Setup and Running

1. Clone the repository to your ROS workspace.
2. Build your ROS workspace.
3. Source the ROS workspace.
4. Run individual nodes using the ros2 run command

## Dependencies

- ROS 2 Humble
- Python 3.10
- `geometry_msgs`
- `turtlesim`
