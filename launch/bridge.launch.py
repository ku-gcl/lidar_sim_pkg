# bridge.launch.py（最小修正）
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gz_topic = '/world/empty/model/lidar_robot/link/laser_link/sensor/mid360/scan/points'
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                # Gazebo -> ROS の一方向ブリッジ（[ は GZ→ROS の意味）
                f'{gz_topic}@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
                # Harmonic(gz) を使う場合は ↓ に差し替え（どちらか片方のみ）
                # f'{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            # SDFの親リンク名は "link" なので、base_link ではなく link に合わせるのが安全
            arguments=['0', '0', '0', '0', '0', '0', 'link', 'laser_link'],
            output='screen',
        ),
    ])
