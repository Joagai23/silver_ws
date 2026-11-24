import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sensors_only = LaunchConfiguration('sensors_only')
    record_sensors = LaunchConfiguration('record_sensors')
    log_dir = LaunchConfiguration('log_dir')

    default_log_dir = os.path.expanduser('~/silver_logs')
    os.makedirs(default_log_dir, exist_ok=True)

    declare_sensors_only = DeclareLaunchArgument(
        'sensors_only',
        default_value='false',
        description='If true, only launches sensors and monitor (skips ros2_control/Dynamixels)',
    )

    declare_record_sensors = DeclareLaunchArgument(
        'record_sensors',
        default_value='true',
        description='If true, records sensor and actuator topics to a rosbag2 (MCAP)',
    )

    declare_log_dir = DeclareLaunchArgument(
        'log_dir',
        default_value=default_log_dir,
        description='Directory where rosbag2 files are stored',
    )

    # 1. SILVER2 Locomotion & ros2_control (skipped if sensors_only:=true)
    silver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('silver'), 'launch', 'silver2.launch.py')
        ),
        condition=UnlessCondition(sensors_only),
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

    # 3. I2C-0 Sensors (LTC2945 Battery Monitor + MS8607 Battery Canister)
    ltc2945_node = Node(
        package='ltc2945',
        executable='ltc2945',
        output='screen',
    )

    ms8607_i2c0_node = Node(
        package='ms8607',
        executable='ms8607_i2c0',
        output='screen',
    )

    # 4. GPIO Sensors (DHT11 Canister Temp/Humidity)
    dht11_node = Node(
        package='dht11',
        executable='dht11',
        output='screen',
    )

    # 5. Live Console Sensor Monitor (always runs, 0.5 Hz = every 2.0s)
    sensor_logger_node = Node(
        package='silver',
        executable='sensor_logger.py',
        parameters=[{'log_rate': 0.5}],
        output='screen',
    )

    # 6. ROS 2 Bag Recorder (MCAP)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_output_path = PathJoinSubstitution([log_dir, f'silver2_bag_{timestamp}'])

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output_path,
            '/ltc2945/vin', '/ltc2945/current', '/ltc2945/power',
            '/ms8607_i2c0/pressure', '/ms8607_i2c0/temperature', '/ms8607_i2c0/humidity',
            '/ms5837/pressure', '/ms5837/temperature', '/ms5837/depth',
            '/bno055/imu', '/bno055/temp', '/bno055/calib_status',
            '/dht11/temperature', '/dht11/humidity',
            '/joint_states', '/cmd_vel', '/pid_position_controller/commands',
        ],
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
        dht11_node,
        sensor_logger_node,
        rosbag_record,
    ])