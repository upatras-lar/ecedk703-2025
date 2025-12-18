from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from lidar_follow_pkg.myarm_utils.math_utils import rot_to_quat
import numpy as np


pkg_name = "lidar_follow_pkg"

# Parameters for the static transformation from myarm base frame to lidar frame
lidar_frame_id = "lidar_frame"
myarm_base_frame_id = "myarm_base_frame"

t_bl = np.array([0.0, 0.0, 0.0])

zrot_bl = np.deg2rad(0.0)
R_bl = np.array([[np.cos(zrot_bl), -np.sin(zrot_bl), 0.0],
                [np.sin(zrot_bl), np.cos(zrot_bl), 0.0],
                [0.0, 0.0, 1.0]])

q_bl = rot_to_quat(R_bl)


def generate_launch_description():
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "myarm_300_pi.urdf"])
    rviz_config_file = PathJoinSubstitution([pkg_share, "config", "rviz", "myarm_lidar_obstacle.rviz"])

    channel_type =  LaunchConfiguration('channel_type', default = 'serial')
    serial_port = LaunchConfiguration('serial_port', default = '/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default = '115200')
    frame_id = LaunchConfiguration('frame_id', default = lidar_frame_id)
    inverted = LaunchConfiguration('inverted', default = 'false')
    angle_compensate = LaunchConfiguration('angle_compensate', default = 'true')
    scan_mode = LaunchConfiguration('scan_mode', default = 'Sensitivity')
    
    return LaunchDescription([

        DeclareLaunchArgument('channel_type', default_value = channel_type, description = 'Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value = serial_port, description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value = serial_baudrate, description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('frame_id', default_value = frame_id, description = 'Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value = inverted, description = 'Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, description = 'Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='Specifying scan mode of lidar'),

        Node(
            package = 'rplidar_ros',
            executable = 'rplidar_node',
            name = 'rplidar_node',
            output = 'screen',
            parameters = [{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
        ),

        Node(
            package = pkg_name,
            executable = "radar_node",
            name = "radar_node",
            output = "screen",
            parameters = [{
                "min_radar_angle": -45.0,
                "max_radar_angle": 20.0,
                "trigger_distance": 0.6,
            }],
        ),

        Node(
            package = pkg_name,
            executable = "myarm_node",
            name = "myarm_node",
            output = "screen",
            parameters = [{
                "port": "/dev/ttyAMA0",
                "baudrate": 115200,
                "speed": 80,
            }],
        ),

        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "myarm_base_to_lidar",
            arguments = [
                str(t_bl[0]), str(t_bl[1]), str(t_bl[2]),
                str(q_bl[0]), str(q_bl[1]), str(q_bl[2]), str(q_bl[3]),
                myarm_base_frame_id,
                lidar_frame_id,
            ],
        ),

        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            output = "screen",
            parameters = [{
                "robot_description": ParameterValue(Command(["cat ", urdf_file]), value_type = str),
            }],
        ),

        Node(
            package = "rviz2",
            executable = "rviz2",
            name = "rviz2",
            output = "screen",
            arguments = ["-d", rviz_config_file],
        ),
    ])