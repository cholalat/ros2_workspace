from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Rutas a los archivos de lanzamiento
    minimal_simulator_launch = os.path.join(
        get_package_share_directory('very_simple_robot_simulator'),
        'launch',
        'minimal_simulator_py.xml'
    )
    openni_simulator_launch = os.path.join(
        get_package_share_directory('very_simple_robot_simulator'),
        'launch',
        'openni_simulator.xml'
    )
    world_state_launch = os.path.join(
        get_package_share_directory('very_simple_robot_simulator'),
        'launch',
        'world_state.xml'
    )

    # Descripción del lanzamiento
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(minimal_simulator_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(openni_simulator_launch)
        ),
        # Descomenta esta línea si necesitas incluir lidar_simulator.xml
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(lidar_simulator_launch)
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_state_launch)
        ),
    ])