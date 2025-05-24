from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Nodos
    obstacle_detector_node = Node(
        package='prueba_5',
        executable='obstacle_detector',
        name='obstacle_detector'
    )

    pose_loader_node = TimerAction(
        period=2.0,  # Retraso de 2 segundos
        actions=[
            Node(
                package='prueba_5',
                executable='pose_loader',
                name='pose_loader'
            )
        ]
    )

    dead_reckoning_p2_node = TimerAction(
        period=4.0,  # Retraso de 4 segundos
        actions=[
            Node(
                package='prueba_5',
                executable='dead_reckoning_p2',
                name='dead_reckoning_p2'
            )
        ]
    )

    # Descripción del lanzamiento
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(run_all_launch)
        ),
        obstacle_detector_node,
        pose_loader_node,
        dead_reckoning_p2_node
    ])