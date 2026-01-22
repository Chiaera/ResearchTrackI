#launch file for the system with both nodes
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rt2_controller',
            executable='node1_ui.py',
            name='user_interface'
        ),
        Node(
            package='rt2_controller',
            executable='node2_controller',
            name='controller'
        ),
    ])

# launch with: ros2 launch rt2_controller system.launch.py
# debug 2 terminals:
#   ros2 run rt2_controller node2_controller
#   ros2 run rt2_controller node1_ui.py