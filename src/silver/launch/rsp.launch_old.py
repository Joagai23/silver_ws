import os
from ament_index_python.packages import get_package_share_directory # Still needed for context, but not for direct pathing below
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindPackageShare # Import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'
    )

    # Declare arguments for package and xacro file names
    robot_description_package_arg = DeclareLaunchArgument(
        'robot_description_package',
        default_value='silver',
        description='Name of the package containing the robot description'
    )

    robot_description_xacro_file_arg = DeclareLaunchArgument(
        'robot_description_xacro_file',
        default_value='silver.xacro',
        description='Name of the xacro file for the robot description'
    )

    # Get LaunchConfiguration objects for the arguments
    robot_description_package = LaunchConfiguration('robot_description_package')
    robot_description_xacro_file = LaunchConfiguration('robot_description_xacro_file')
    use_sim_time_lc = LaunchConfiguration('use_sim_time') # Renamed to avoid confusion
    use_ros2_control_lc = LaunchConfiguration('use_ros2_control') # Renamed to avoid confusion


    # Construct the path to the xacro file using PathJoinSubstitution and FindPackageShare.
    # FindPackageShare is a substitution that resolves the package share directory.
    xacro_file_path = PathJoinSubstitution([
        FindPackageShare(robot_description_package), # This is the key change!
        'description', # Assuming your xacro file is in a 'description' directory
        robot_description_xacro_file
    ])

    # Process the xacro file to generate the URDF content
    robot_description_content = ParameterValue(
        Command([
            'xacro',
            ' ',
            xacro_file_path, # Pass the PathJoinSubstitution directly
            ' ',
            'use_ros2_control:=', use_ros2_control_lc, # Pass LaunchConfiguration directly to Command list
            ' ',
            'sim_mode:=', use_sim_time_lc # Pass LaunchConfiguration directly to Command list
        ]),
        value_type=str
    )

    # Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_lc,
            'robot_description': robot_description_content
        }]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        use_sim_time,
        use_ros2_control,
        robot_description_package_arg, # Add argument declarations to the LaunchDescription
        robot_description_xacro_file_arg, # Add argument declarations to the LaunchDescription
        node_robot_state_publisher
    ])