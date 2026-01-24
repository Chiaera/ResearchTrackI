# ASSIGNMENT 2
This project is a ROS 2 package composed of **two nodes** and **two services**  that control a robot to move in a space with obstacles without colliding with them.

The system allows the user to drive the robot around. When the robot gets "too close" to an obstacle (detected by the laser scanner), it performs an **automatic 1-second backward motion** to return to a safe position.

The user can:
- set linear and angular velocities
- change the obstacle detection threshold
- get the average velocities of the last 5 inputs

<br>

## Nodes
1. ### `node1_ui` - Python
   This node provides a **keyboard-based menu interface** to control the robot:
   - read keyboard input (single key press, no Enter required)
     ```
     motion
     f - Forward
     s - Backward
     e - Rotate left
     r - Rotate right
     d - Stop

     settings
     t - Set new threshold
     y - Get averages of last 5 velocities
     u - Set linear speed
     i - Set angular speed
     
     h - help
     q - Quit
     ```
   - convert input to command
   - publish velocity command to `/cmd_vel_input`
   - call services `SetThreshold()` and `GetAverages()`

2. ### `node2_controller` - C++
   This node handles **robot control** and **obstacle avoidance**:

   **Robot control**
   - subscribe to
        - `/cmd_vel_input` (user commands)
        - `/scan` (laser scanner data)
   - publish to
        - `/cmd_vel` (safe velocity commands)
        - `/obstacle_info` (obstacle information)
   - provide service to
        - `/set_threshold` (change obstacle detection threshold)
        - `/get_averages` (Get average velocities of last 5 inputs).
          
     **Obstacle avoidance**
     The laser scanner divides the space into zones:
     - **Front zone** (-30° to +30°)
     - **Left zone** (+30° to +90°)
     - **Right zone** (-90° to -30°)
     - **Back zone** (-180° to -120° and +120° to +180°)
     The robot behaves differently depending on the area:
     1. **Forward motion, if obstacle is in front/lateral zone**: backward motion of 1 second, the robot will return in the previously position
     2. **Forward command, if obstacle is in forward hemisphere**: the forward motion is blocked, the rebot can only rotate
     3. **Backward command, if obstacle is behind**: the backward motion is blocked

## Services
  1. ### `GetAverages`
     Computes the *averages of the last 5 velocities (linear and angular)*, if the command are less than 5, the service will return the average anyway but it will specify how many values are considered.
  2. ### `SetThreshold`
     Changes the obstacle detection threshold value (in meters).

## Custom Message
### `ObstacleInfo`
Publish the information about the obstacle into `/obstacle_info` topic:
- minimun distance to any obstacle
- direction od the closest obstacle
- current threshold value

---

## Requirements
Operating System: Ubuntu (suggested: 24.04)
ROS 2: suggested distribution Jazzy
Standard ROS 3 tools:
- `colcon`
- Gazebo and Rviz
- `xterm` terminal

To install **gazebo anz Rviz** and the simulation enviornment:
```
# install Gazebo Harmonic (per Jazzy)
sudo apt update
sudo apt install ros-jazzy-ros-gz

# install RViz2
sudo apt install ros-jazzy-rviz2

# clone the professor repository of the Research Track course
cd ~/ros2_ws/src
git clone https://github.com/CarmineD8/bme_gazebo_sensors.git
```

To install the **xterm** terminal:
```
sudo apt install xterm
```
---

## Workspace Setup
In the desired directory, create the workspace:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
clone the `ResearchTrackI` repository
```
gh repo clone Chiaera/ResearchTrackI
```
The final structure should look like this:
```
.
└── ros2_ws
      └── src
          ├── bme_gazebo_sensors   # simulation package
          │   ├── CMakeLists.txt
          │   ├── config
          │   │   └── ekf.yaml
          │   ├── launch
          │   │   ├── spawn_robot.launch.py
          │   │   └── world.launch.py
          │   ├── meshes
          │   │   ├── lidar.dae
          │   │   ├── mogi_bot.dae
          │   │   └── wheel.dae
          │   ├── package.xml
          │   ├── rviz
          │   │   ├── gps.rviz
          │   │   ├── rviz.rviz
          │   │   └── urdf.rviz
          │   ├── urdf
          │   │   ├── materials.xacro
          │   │   ├── mogi_bot.gazebo
          │   │   └── mogi_bot.urdf
          │   └── worlds
          │       ├── home.sdf
          │       ├── my.sdf
          │       ├── my_world.sdf
          │       └── world.sdf
          |
          └── ResearchTrackI
              └── assignment1
              │       └── (*)
              └──  assignment2   # assignment directory
                      ├── rt2_controller
                      │   ├── CMakeLists.txt
                      │   ├── include
                      │   │   └── rt2_controller
                      │   ├── launch
                      │   │   └── assignment2.launch.py
                      │   ├── package.xml
                      │   ├── scripts
                      │   │   └── node1_ui.py
                      │   └── src
                      │       └── node2_controller.cpp
                      └── rt2_interfaces
                          ├── CMakeLists.txt
                          ├── include
                          │   └── rt2_interfaces
                          ├── msg
                          │   └── ObstacleInfo.msg
                          ├── package.xml
                          ├── src
                          └── srv
                              ├── GetAverages.srv
                              └── SetThreshold.srv
```
*( * ) for the **assignment1** structure see the  `assignment1` directory*.

---

## Execute the file
Using the launch file:
```
# from the workspace root
cd ~/ros2_ws

# build the enviornment
colcon build

# active the enviornment
source install/setup.bash

# Launch the whole assignment
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```
This will open 3 xterm windows:
- Gazebo + RViz (simulation)
- Controller node (logs)
- UI node (keyboard control)

You can also launch every node manually (3 separate terminals):
Terminal 1:
```
colcon build
source install/setup.bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```

Terminal 2:
```
source install/setup.bash
ros2 run rt2_controller node2_controller
```

Terminal 3:
```
source install/setup.bash
ros2 run rt2_controller node1_ui.py
```
Once launched, follow the UI prompts in the terminal to control the robot.
