from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/lidar_robot/laser_link/mid360@'
                'gz.msgs.PointCloudPacked[sensor_msgs/msg/PointCloud2'
            ],
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0','0','0','0','0','0',
                'base_link','laser_link'
            ],
            output='screen'),
    ])
