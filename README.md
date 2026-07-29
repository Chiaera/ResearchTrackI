# ROS2 Collision Avoidance (Turtlesim and Gazebo)

This repository contains the two assignments of the course *Research Track I*:

1. **Turtlesim Simulation**  
   Two ROS2 nodes that control two turtles in turtlesim and prevent collisions between them and with the borders.

2. **Gazebo Simulation**  
   Two ROS2 nodes and two services that control a robot moving in an environment with obstacles, avoiding collisions.

Each assignment is organized in its own dedicated folder, with its own README and specifications.

---

## Running one assignment at a time

Inside the workspace, the `assignment1` and `assignment2` folders each contain ROS2 packages.  
To avoid building both assignments at the same time, we use the `COLCON_IGNORE` file:  
any folder containing this file will be ignored by `colcon build`.

By default, the `COLCON_IGNORE` file is placed inside `assignment1`.  
To run one assignment or the other, simply move (or recreate) the file in the appropriate folder.

### Switch between assignments

```bash
# Run assignment 1 — ignore assignment2
touch src/prova_RT/assignment2/COLCON_IGNORE
rm src/prova_RT/assignment1/COLCON_IGNORE 2>/dev/null

# Run assignment 2 — ignore assignment1
touch src/prova_RT/assignment1/COLCON_IGNORE
rm src/prova_RT/assignment2/COLCON_IGNORE 2>/dev/null
