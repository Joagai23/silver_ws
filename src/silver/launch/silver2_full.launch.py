import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # SILVER2 Locomotion & ros2_control
    silver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('silver'), 'launch', 'silver2.launch.py')
        )
    )

    # --- I2C-1 Sensors ---

    # BNO055 IMU
    bno055_config = os.path.join(
        get_package_share_directory('bno055'), 'config', 'bno055_params_i2c.yaml'
    )
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        parameters=[bno055_config],
        output='screen',
    )

    # MS5837 External Depth Sensor (0x76 on i2c-1)
    ms5837_node = Node(
        package='ms5837',
        executable='ms5837',
        output='screen',
    )

    # --- I2C-0 Sensors ---

    # LTC2945 Battery Node (0x67 on i2c-0)
    ltc2945_node = Node(
        package="ltc2945",
        executable="ltc2945",
        output='screen'
    )

    # MS8607 Pressure-Temperature-Humidity Node (0x76, 0x40 on i2c-0)
    ms8607_i2c0_node = Node(
        package='ms8607',
        executable='ms8607_i2c0',
        output='screen',
    )

    # NOTE: ms8607_i2c1 shares 0x76 on i2c-1 with ms5837.
    # Leave disabled if ms5837 is connected on i2c-1.
    # ms8607_i2c1_node = Node(
    #     package='ms8607',
    #     executable='ms8607_i2c1',
    #     output='screen',
    # )

    return LaunchDescription([
        silver_launch,
        bno055_node,
        ltc2945_node,
        ms5837_node,
        ms8607_i2c0_node
    ])