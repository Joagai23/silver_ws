import os
import sys
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

# --- Hardware Detection Logic ---
sys.path.append('/home/silver/silver_ws/src/dynamixel/debug_scripts')
try:
    from dynamixel_controller import get_hardware_status
except ImportError:
    print(f"ERROR: Could not import from dynamixel_controller.py: {e}")
    def get_hardware_status(): return []

def get_active_legs():
    return get_hardware_status()

def generate_launch_description():

    # Specify the name of the package
    pkg_name = 'silver'

    # Detect Hardware
    active_legs = get_active_legs()
    print(f"Launching SILVER2 with active legs: {active_legs}")

    # Process robot description
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'silver.xacro')
    doc = xacro.process_file(xacro_file, mappings={'sim_mode': 'false', 'use_ros2_control': 'true'})
    robot_description_raw = doc.toprettyxml(indent='  ')

    # Specify controllers configurations
    controller_config = os.path.join(
        get_package_share_directory(pkg_name), "config", "silver2_controllers.yaml"
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name),'launch','rsp.launch.py'
        )]), 
        launch_arguments={'sim_mode': 'false', 'use_ros2_control': 'true'}.items()
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_raw}, 
            controller_config
        ],
        output="screen"
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    pid_position_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'pid_position_controller'],
        output='screen'
    )

    delayed_pid_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[pid_position_controller_spawner],
        )
    )

    # Run the node
    return LaunchDescription([
        rsp,
        controller_manager,
        joint_state_broadcaster_spawner,
        delayed_pid_position_controller_spawner
    ])


