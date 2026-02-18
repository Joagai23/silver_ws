from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('sim_mode')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    declare_use_sim_time = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Enable ROS 2 control if true'
    )

    # Path to xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('silver'),  # Replace with your actual package name
        'description',
        'silver.xacro'
    ])

    # Process xacro to URDF
    robot_description_content = Command(['xacro ', xacro_file,
                                         ' use_sim_time:=', use_sim_time,
                                         ' use_ros2_control:=', use_ros2_control])

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_ros2_control,
        robot_state_publisher_node
    ])
