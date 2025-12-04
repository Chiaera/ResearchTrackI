# ASSIGNMENT 1

This project is a ROS 2 package composed of **two nodes** that control two turtles in `turtlesim` and prevent collisions between them and with the borders.

## Nodes

### 1. UI Node (`node1_ui`)

This node provides a simple **text-based user interface**.

- Lets the user:
  - select which turtle to move (`turtle1` or `turtle2`)
  - insert linear and angular velocities
- Publishes velocity commands to the selected turtle for **1 second**
- Publishes the **ID of the active turtle** on the `/active_turtle` topic

### 2. Distance Node (`node2_distance`)

This node is responsible for:

- Computing the **distance between the two turtles** and publishing it on `/turtles_distance` (`std_msgs/Float32`)
- Detecting when the turtles are:
  - too close to each other
  - too close to the borders
- Implementing the **avoidance and stop behaviours**:
  - avoidance manoeuvre in the *orange* zone
  - stop / retreat manoeuvre in the *red* zone

A small helper node, `turtle_spawn.py`, is also used to spawn `turtle2` at a custom position.


---

## Requirements

- **Operating System**: Ubuntu (suggested: **24.04**)
- **ROS 2**: suggested distribution **Jazzy**
- Standard ROS 2 tools:
  - `colcon`
  - `turtlesim` package

---

## Workspace Setup

From your preferred directory, create the workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
and clone the ResearchTrackI repository
```bash
gh repo clone Chiaera/ResearchTrackI
```

The final structure should look like this:
```bash
.
└── ros2_ws
└── src
    └── ResearchTrackI
        └── assignment1
            ├── launch_assignment1.sh
            ├── node1_ui
            │   ├── package.xml
            │   ├── resource
            │   │   └── node1_ui
            │   ├── script
            │   │   ├── __init__.py
            │   │   ├── node_ui.py
            │   │   └── turtle_spawn.py
            │   ├── setup.cfg
            │   ├── setup.py
            │   └── test
            │       ├── test_copyright.py
            │       ├── test_flake8.py
            │       └── test_pep257.py
            ├── node2_distance
            │   ├── CMakeLists.txt
            │   ├── include
            │   │   └── node2_distance
            │   ├── launch
            │   │   └── assignment_launch.py
            │   ├── package.xml
            │   └── src
            │       └── node_distance.cpp
            └── README.md
```

---

## Execute the file

First, make sure to launch the script executable (only once):
```bash
#from the assignment1 directory
cd ~/ros2_ws/src/ResearchTrackI/assignment1
chmod +x launch_assignment1.sh
```

Now you can run the project:
```bash
#from the workspace root
cd ~/ros2_ws

# Launch the whole assignment
./src/ResearchTrackI/assignment1/launch_assignment1.sh
```

Once launched, follow the UI prompts in the terminal to control the turtles.
