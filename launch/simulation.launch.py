# simulation.launch.py
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パッケージ共有ディレクトリを参照
    pkg_share = FindPackageShare('lidar_sim_pkg')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    # print(ros_gz_sim_share)

    # ワールドファイルとモデルディレクトリのパス
    # world_file = PathJoinSubstitution([pkg_share, 'worlds', 'lidar_sim_world.sdf'])
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'default.sdf'])
    resource_path = PathJoinSubstitution([pkg_share, 'models'])

    return LaunchDescription([
        # Gazebo のリソースパスにモデルを追加
        # SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),

        # ros_gz_sim の汎用 launch を利用して Gazebo 起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={
                'gz_args': world_file,
                'on_exit_shutdown': 'True'
            }.items(),
        ),
    ])