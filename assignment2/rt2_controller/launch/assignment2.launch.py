from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    #launch gazebo 
    pkg_bme = get_package_share_directory('bme_gazebo_sensors')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme, 'launch', 'spawn_robot.launch.py')
        )
    )

    #node2: controller
    controller_node = Node(
        package='rt2_controller',
        executable='node2_controller',
        name='node2_controller',
        output='screen',
        prefix='xterm -hold -e'
    )

    #node1: UI
    ui_node = Node(
        package='rt2_controller',
        executable='node1_ui.py',
        name='node1_ui',
        output='screen',
        prefix='xterm -hold -e'
    )

    return LaunchDescription([
        gazebo_launch,
        controller_node,
        ui_node
    ])


#ELSE:
#terminal 1:
#   colcon build
#   source install/setup.bash
#   ros2 launch bme_gazebo_sensors spawn_robot.launch.py
#terminal 2:
#   source install/setup.bash
#   ros2 run rt2_controller node2_controller
#terminal 3:
#   source install/setup.bash
#   ros2 run rt2_controller node1_ui.py