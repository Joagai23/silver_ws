import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments to toggle sensors
    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true', description='Launch BNO055 IMU node'
    )
    use_dht11_arg = DeclareLaunchArgument(
        'use_dht11', default_value='true', description='Launch DHT11 humidity/temp nodes'
    )

    use_imu = LaunchConfiguration('use_imu')
    use_dht11 = LaunchConfiguration('use_dht11')

    # 1. SILVER2 Locomotion & ros2_control
    silver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('silver'), 'launch', 'silver2.launch.py')
        )
    )

    # 2. BNO055 IMU Node
    bno055_config = os.path.join(
        get_package_share_directory('bno055'), 'config', 'bno055_params_i2c.yaml'
    )
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        parameters=[bno055_config],
        output='screen',
        condition=IfCondition(use_imu),
    )

    # 3. DHT11 Humidity & Temperature Nodes
    dht11_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('dht11'), 'launch', 'dht11_launch.py')
        ),
        condition=IfCondition(use_dht11),
    )

    return LaunchDescription([
        use_imu_arg,
        use_dht11_arg,
        silver_launch,
        bno055_node,
        dht11_launch,
    ])