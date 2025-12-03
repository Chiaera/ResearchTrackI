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


PER ADESSO - UI
    - colcon build
    - source install/setup.bash
    - ros2 run turtlesim turtlesim_node | ros2 run node1_ui turtle_spawn | ros2 run node1_ui node_ui