from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        #turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        #turtle_spawn
        Node(
            package='node1_ui',
            executable='turtle_spawn',
            name='turtle_spawn',
            output='screen'
        ),

        #node2: DISTANCE
        Node(
            package='node2_distance',
            executable='node_distance',
            name='node_distance',
            output='screen'
        ),
    ]) 