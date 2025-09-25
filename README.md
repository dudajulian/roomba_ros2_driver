# roomba_ros2_driver

A ROS2 Jazzy Node which works as a bridge between the iRobot Roomba SCI messages and the ROS2 messages. After connection the Roomba to an RPi or other sigle board computer, 
you can run this node which subscribes to `geometry_msgs/msg/Twist` commands. Also sensor data is read and publish as well as the odometry computed and published.

## Features

- Listens to ROS2 velocity commands (`/cmd_vel`)
- Translates and sends commands to the Roomba robot
- Odometry computed and published
- Designed for integration with other ROS2 navigation and control stacks


## Requirements (Hardware)
- iRobot Roomba with SCI (Roomba 400 series (with OSMO or firmware update), Roomba 500/600/700/800/900 Series)
- Raspberry Pi 4/5 (RPi)
- Logic Level schifter (3.3V-5V)
- UART-to-TTL Adapter
- DC-DC Converterd (to 5V)
- Wires

## Setup Guide
- ROS2 Humble for RPi4 / ROS2 Jazzy for RPi5
- ros2 jazzy base
- rpi5
- ubunut 24.04 server

### additional ros2 packages
-  sudo apt install ros-jazzy-tf-transformations

## Usage

1. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select roomba_ros2_driver
   source install/setup.bash
   ```

2. **Launch the node:**

   - **Directly:**
     ```bash
     ros2 run roomba_ros2_driver roomba_ros2_driver_node
     ```

   - **Using the launch file:**
     ```bash
     ros2 launch roomba_ros2_driver roomba_ros2_driver.launch.py
     ```

3. **Send velocity commands:**
   Publish to the `/cmd_vel` topic using `ros2 topic pub` or your own ROS2 navigation stack:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
   ```

   You can also control the robot interactively using the [teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard) package:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   This will send velocity commands to `/cmd_vel` using your keyboard.


## TODO

- [ ] Fix the TODOs in the code
- [ ] Create URDF model that can be used for Simulation (using Fusion 360)


## License

MIT License

## Authors

- [Your Name] (replace with actual author)
