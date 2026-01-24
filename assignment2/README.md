# ASSIGNMENT 2
This project is a ROS 2 package composed of **two nodes** and **two services**  that control a robot to move in a space with obstacles without colliding with them.

The system allows the user to drive the robot around. When the robot gets "too close" to an obstacle (detected by the laser scanner), it performs an **automatic 1-second backward motion** to return to a safe position.

The user can:
- set linear and angular velocities
- change the obstacle detection threshold
- get the average velocities of the last 5 commands

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
   - convert input into commands
   - publish velocity commands to `/cmd_vel_input`
   - call services `SetThreshold()` and `GetAverages()`

2. ### `node2_controller` - C++
   This node handles **robot control** and **obstacle avoidance**:

   #### Robot control
   - subscribe to
        - `/cmd_vel_input` (user commands)
        - `/scan` (laser scanner data)
   - publish to
        - `/cmd_vel` (safe velocity commands)
        - `/obstacle_info` (obstacle information)
   - provide service to
        - `/set_threshold` (update obstacle detection threshold)
        - `/get_averages` (compute average velocities of last 5 inputs).

   <br>
          
   #### Obstacle avoidance
   The laser scanner divides into angular zones:
     - **Front zone** (-30° to +30°)
     - **Left zone** (+30° to +90°)
     - **Right zone** (-90° to -30°)
     - **Back zone** (-180° to -120° and +120° to +180°)
       
   The controller applies the following safety rules:
   - **Forward motion with obstacle in the forward hemisphere**: automatic *backward motion* for 1 second to return to a safe position
   - **Forward command while an obstacle remains in the forward zone**: forward motion is blocked, the robot can *only rotate*
   - **Backward command with obstacle behind the robot**: backward motion is blocked

<br>
    
Only safe velocity commands are published to `/cmd_vel`.

## Services
  1. ### `GetAverages`
     Computes the *averages of the last 5 velocities (linear and angular)*.
     If fewer than 5 commands are available, the average is computed over the existing inputs and the number of samples used is reported.
  3. ### `SetThreshold`
     Updates the obstacle detection threshold value (in meters).

## Custom Message
### `ObstacleInfo`
Publish the information about the obstacle on `/obstacle_info` topic:
- minimum distance to the closest obstacle
- direction of the closest obstacle (**front**, **left**, **right**, **back**)
- current threshold value

---

## Requirements
Operating System: Ubuntu (suggested: 24.04)
ROS 2: suggested distribution Jazzy
Standard ROS 3 tools:
- `colcon`
- Gazebo
- Rviz
- `xterm` terminal

To install **Gazebo anz Rviz**:
```
# install Gazebo Harmonic (per Jazzy)
sudo apt update
sudo apt install ros-jazzy-ros-gz

# install RViz2
sudo apt install ros-jazzy-rviz2
```

To clone the **simulation package**:
```
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
Create the workspace:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

clone the `ResearchTrackI` repository:
```
gh repo clone Chiaera/ResearchTrackI
```

The final structure should look like this:
```
.
└── ros2_ws
      └── src
          ├── bme_gazebo_sensors   # simulation package
          |
          └── ResearchTrackI
              └── assignment1   # assignment 1 directory
              │       └── (*)
              |
              └──  assignment2   # assignment 2 directory
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
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```
This opens 3 `xterm` windows:
- Gazebo + RViz (simulation)
- Controller node (logs)
- UI node (keyboard control)

You can also launch the nodes manually (3 separate terminals):
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
Follow the UI instructions to control the robot.
