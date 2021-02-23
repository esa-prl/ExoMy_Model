import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
# namespace_ = 'exomy'

def generate_launch_description():
    # Get paths to config files
    sim_config = os.path.join(get_package_share_directory('exomy_sim'),'rviz/simulation.rviz')
    world = os.path.join(get_package_share_directory('exomy_sim'), 'worlds/mars_yard.world')

    urdf_file = os.path.join(get_package_share_directory('exomy_sim'),'models/exomy_model/exomy_model.urdf')
    # Some packages require the path to the urdf file, others require the opened file:
    with open(urdf_file, 'r') as infp:
            robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true')


    robot = Node(
        package='exomy',
        executable='robot_node',
        name='robot_node',
        output='screen'
    )
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    gamepad = Node(
        package='exomy',
        executable='gamepad_parser_node',
        name='gamepad_parser_node',
        output='screen'
    )

    joint_command_node = Node(
        package='exomy_sim',
        executable='joint_command_node',
        name='joint_command_node',
        output='screen',

    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
        }],
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # output='screen',
        output={'stdout':'log'},
        arguments=['-d', sim_config],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

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

        # Declare launch arguments
        declare_use_sim_time_cmd,

        # Declare nodes
        robot,
        gamepad,
        joy,
        joint_command_node,
        robot_state_publisher,
        rviz2,
        gazebo,
        spawn_rover,
    ])
