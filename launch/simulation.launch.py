import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_sim_pkg')
    world_path = os.path.join(pkg_share, 'worlds', 'lidar_sim_world.sdf')
    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='gzserver',
            arguments=[world_path],
            output='screen'),
        Node(
            package='ros_gz_sim',
            executable='gzclient',
            output='screen'),
    ])
