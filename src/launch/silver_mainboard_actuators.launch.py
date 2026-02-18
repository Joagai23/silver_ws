import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # Declare Launch Arguments
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode', default_value='False', description='Enable simulation mode (true/false)'
    )
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control', default_value='true', description='Use ros2_control interface'
    )

    # Config values from arguments
    use_sim_time = LaunchConfiguration('sim_mode')
    ros2_control_enabled = LaunchConfiguration('use_ros2_control')

    # Get package share directory
    silver_share_dir = get_package_share_directory('silver')

    # Path to xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('silver'),
        'description',
        'silver.xacro'
    ])

    # Process xacro to URDF
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_sim_time:=', use_sim_time,
        ' use_ros2_control:=', ros2_control_enabled
    ])

    # Controller config paths
    controller_config_path = os.path.join(silver_share_dir, "config", "silver_controllers.yaml")
    gaz_ros2_control_use_sim_config_path = os.path.join(silver_share_dir, "config", "gaz_ros2_control_use_sim.yaml")

    # Include RSP launch
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(silver_share_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'sim_mode': use_sim_time,
            'use_ros2_control': ros2_control_enabled
        }.items()
    )

    # Base controller manager params
    base_params = [
        {"robot_description": robot_description_content},
        controller_config_path,
        {'sim_mode': use_sim_time}
    ]

    # Controller Manager for real hardware
    controller_manager_real = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=base_params,
        output="screen",
        condition=UnlessCondition(use_sim_time)
    )

    # Controller Manager for simulation
    controller_manager_sim = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[*base_params, gaz_ros2_control_use_sim_config_path],
        output="screen",
        condition=IfCondition(use_sim_time)
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        sim_mode_arg,
        use_ros2_control_arg,
        rsp,
        controller_manager_real,
        controller_manager_sim,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner
    ])
