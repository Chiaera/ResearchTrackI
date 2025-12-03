ASSIGNMENT 1
- UI (node1)
    - textual Interface: 
          robot selection (turtle1 or turtle2)
          velocity input
    - execution for 1 second
    - new input -> loop

- DISTANCE (node2)
    - distance calculation between turtle1 e turtle2
          publish on a topic (msg: std_msgs/Float32)
          stop turtles if they are 'too close' (threshold)
          stop turtles if they are too close to the boundaries


SET UP THE WORKSPACE
    - create a workspace
        mkdir -p ros2_ws/src
        cd ros2_ws/src
    
    - copy repository ResearchTrackI in src
    - the final project structure should be
        ros2_ws
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

EXECUTE ASSIGNMENT
    - in the workspace directory
        cd ros2_ws
    - launch script
        ./src/ResearchTrackI/assignment1/launch_assignment1.sh


