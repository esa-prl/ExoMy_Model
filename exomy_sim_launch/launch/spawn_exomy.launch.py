import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get paths to config files
    urdf_file = os.path.join(get_package_share_directory('exomy_sim'),'models/exomy_model/exomy_model.urdf')

    # Spawn rover
    spawn_rover = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        namespace='',
        arguments=['-entity',
                   'exomy',
                   '-x', '1.5', '-y', '1', '-z', '2',
                   '-file', urdf_file,
                   '-reference_frame', 'world']
    )

    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Declare nodes
        spawn_rover,
    ])
