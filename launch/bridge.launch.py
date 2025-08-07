# bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Gazebo Sim から ROS 2 へのポイントクラウドブリッジ
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/model/lidar_robot/laser_link/mid360@gz.msgs.PointCloudPacked[sensor_msgs/msg/PointCloud2]'
            ],
            output='screen',
        ),
        # base_link と laser_link の静的変換をパブリッシュ
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
            output='screen',
        ),
    ])
