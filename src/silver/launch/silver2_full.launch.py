import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sensors_only = LaunchConfiguration('sensors_only')
    record_sensors = LaunchConfiguration('record_sensors')
    log_dir = LaunchConfiguration('log_dir')

    declare_sensors_only = DeclareLaunchArgument(
        'sensors_only',
        default_value='false',
        description='If true, only launches I2C sensors and logger (skips ros2_control/Dynamixels)',
    )

    declare_record_sensors = DeclareLaunchArgument(
        'record_sensors',
        default_value='true',
        description='If true, runs sensor_logger.py to save sensor telemetry to CSV',
    )

    declare_log_dir = DeclareLaunchArgument(
        'log_dir',
        default_value=os.path.expanduser('~/silver_logs'),
        description='Directory where CSV sensor logs and rosbag2 files are stored',
    )
    
    # 1. SILVER2 Locomotion & ros2_control (skipped if sensors_only:=true)
    silver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('silver'), 'launch', 'silver2.launch.py')
        ),
        condition=UnlessCondition(sensors_only)
    )

    # 2. I2C-1 Sensors (BNO055 IMU + MS5837 Bar30 Depth)
    bno055_config = os.path.join(
        get_package_share_directory('bno055'), 'config', 'bno055_params_i2c.yaml'
    )
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        parameters=[bno055_config],
        output='screen',
    )

    ms5837_node = Node(
        package='ms5837',
        executable='ms5837',
        output='screen',
    )

    # 3. I2C-0 Sensors (LTC2945 Battery Monitor + MS8607 Battery Canister P/T/RH)
    ltc2945_node = Node(
        package="ltc2945",
        executable="ltc2945",
        output='screen'
    )

    ms8607_i2c0_node = Node(
        package='ms8607',
        executable='ms8607_i2c0',
        output='screen',
    )

    # 4. CSV Sensor Logger Node
    sensor_logger_node = Node(
        package='silver',
        executable='sensor_logger.py',
        parameters=[{'log_dir': log_dir, 'log_rate': 2.0}],
        output='screen',
        condition=IfCondition(record_sensors),
    )

    return LaunchDescription([
        declare_sensors_only,
        declare_record_sensors,
        declare_log_dir,
        silver_launch,
        bno055_node,
        ms5837_node,
        ltc2945_node,
        ms8607_i2c0_node,
        sensor_logger_node,
    ])