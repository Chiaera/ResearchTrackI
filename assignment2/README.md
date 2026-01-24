# ASSIGNMENT 2
This project is a ROS 2 package composed of **two nodes** and **two services**  that control a robot to move in a space with obstacles without collide with them.

It allows the user to drive the robot around, when the robot is "too close" to an obstacle (traced by the laser), the robot return in the position of 1 second before (*backward motion*).
The user can set the linear velcity (it can change the default one), the angular velocity and the threshold value, it can also get the average linear and angular velocity of the most recent 5 inputs.

<br>

## Nodes
1. ### `node1_ui` - Python
   This node provides the **menu interfaced** used to control the robot
   - read the input from the keyboard
     ```
     f - Forward
     s - Backward
     e - Rotate left
     r - Rotate right
     d - Stop
     
     t - Set new threshold
     y - Get averages of last 5 velocities
     u - Set linear speed
     i - Set angular speed
     
     h - help
     q - Quit
     ```
   - turn the input into command
   - publish command to `controller_node`
   - call services to `SetThreshold()` and `GetAverages()`

2. ### `node2_controller` - C++
   This node is resposable fot the **robot control** and **obstacles avoidance**
   - subscribe to user commands, laser scan,
   - publish safe commands to robot, obstacle avoidance,
   - provide service to set obstacle threshold and get average velocities.
     The laser scan the **front**, **lateral** and **back** zone, the robot is the area under the threshould value, its behaviour will depend on the zone:
     - if an obstacle is on **front** or **lateral** the robot will start a backward motion for 1 seconds, returning to a previously position;
     - if an obstacle is on **back** the robot will stop.

## Services
  1. ### `GetAverages`
     It rapresents the response to computing the *averages of the last 5 velocities (linear and angular)*, if the command are less than 5, the service will return the average anyway but it will specify how many values are considered.
  2. ### `SetThreshold`
     It represents the response to *change the threshold value*.
     


---

## Requirements
Operating System: Ubuntu (suggested: 24.04)
ROS 2: suggested distribution Jazzy
Standard ROS 3 tools:
- colcon
- Gazebo and Rviz package
- xterm terminal

For **gazebo anz Rviz package**:
```
# From the ~/ros2_ws/src path
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
As the **Assignemnt 1**, from your preferred directory, create the workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
and clone the `ResearchTrackI` repository
```
gh repo clone Chiaera/ResearchTrackI
```
The final structure should look like this:
```
.
└── ros2_ws
└── src
    └── ResearchTrackI
        └── assignment1
        │       └── (*)
        └──  assignment2
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
To run the assignment from the *LaunchFile*:
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

For launch every node singular:
```
# in the FIRST terminal:
colcon build
source install/setup.bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py

# in the SECOND terminal:
source install/setup.bash
ros2 run rt2_controller node2_controller

# in the THIRD terminal: 
source install/setup.bash
ros2 run rt2_controller node1_ui.py
```
Once launched, follow the UI prompts in the terminal to control the robot.
