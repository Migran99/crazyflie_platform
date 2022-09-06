from os.path import join

import launch
from launch import LaunchDescription, conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    config = join(
        get_package_share_directory('crazyflie_platform'),
        'config',
        'control_modes.yaml'
    )
    DRONE_ID = os.environ['AEROSTACK2_SIMULATION_DRONE_ID']
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=DRONE_ID),
        DeclareLaunchArgument('mass', default_value='0.029'),
        DeclareLaunchArgument('max_thrust', default_value='0.0'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        DeclareLaunchArgument('external_odom',default_value='true'),
        DeclareLaunchArgument('drone_URI',default_value='radio://0/80/250K/E7E7E7E7E7'),
        DeclareLaunchArgument('external_odom_topic',default_value='external_odom'),
        DeclareLaunchArgument('simulation_mode',default_value='false'),
        DeclareLaunchArgument('min_thrust',default_value='0.0'),
        DeclareLaunchArgument('controller_type',default_value='1'),
        DeclareLaunchArgument('estimator_type',default_value='1'),
        DeclareLaunchArgument('log_level',default_value='INFO'),
        # if is not in simulation
        Node(
            package="crazyflie_platform",
            executable="crazyflie_platform_node",
            name="platform",
            namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[
                {"mass": LaunchConfiguration('mass'),
                "max_thrust": LaunchConfiguration('max_thrust'),
                "control_modes_file": LaunchConfiguration('control_modes_file'),
                "external_odom" : LaunchConfiguration('external_odom'),
                "drone_URI" : LaunchConfiguration('drone_URI'),
                "external_odom_topic" : LaunchConfiguration('external_odom_topic'),
                "min_thrust": LaunchConfiguration('min_thrust'),
                "simulation_mode": LaunchConfiguration('simulation_mode'),
                "controller_type": LaunchConfiguration('controller_type'),
                "estimator_type": LaunchConfiguration('estimator_type'),
                }],
            arguments=['--ros-args','--log-level',LaunchConfiguration('log_level')]
            #remappings=[("sensor_measurements/odometry", "self_localization/odom")],
        )    
    ])