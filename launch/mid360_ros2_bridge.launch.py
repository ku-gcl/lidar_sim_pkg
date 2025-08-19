from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gz_topic = "/world/helipad_lidar/model/wrscopter_lidar_0/link/mid360_link/sensor/lidar/scan/points"
    pc2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        remappings=[(gz_topic, "/livox/lidar")],
        output="screen",
    )

    return LaunchDescription([
        pc2_bridge
    ])
