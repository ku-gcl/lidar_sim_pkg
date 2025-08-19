from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gz_topic = '/lidar/points'
    ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Gazebo -> ROS 2
            f'{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[(gz_topic, "/livox/lidar")],
        output='screen',
    )
    
    # tf2_ros = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_laser',
    #     # SDFの親リンク名は "link" なので、base_link ではなく link に合わせるのが安全
    #     arguments=['0', '0', '0', '0', '0', '0', 'link', 'laser_link'],
    #     output='screen',
    # )
    
    return LaunchDescription([
        ros_bridge,
        # tf2_ros,
    ])
