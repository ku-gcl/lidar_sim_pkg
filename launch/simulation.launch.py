# simulation.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パッケージ共有ディレクトリを参照
    pkg_share = FindPackageShare('lidar_sim_pkg')

    # ワールドファイルとモデルディレクトリのパス
    world = PathJoinSubstitution([pkg_share, 'worlds', 'sensor_tutorial.sdf'])
    
    # SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
    set_res_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', 
        value=PathJoinSubstitution([pkg_share, "worlds"])
    )
    
    gz_sim = ExecuteProcess(
        cmd=["ign", "gazebo", world],
        output="screen"
    )
    
    lidar_node = Node(
        package="lidar_sim_pkg",
        executable="lidar_node",
        name="lidar_node",
        output="screen",
    )

    return LaunchDescription([set_res_path, gz_sim, lidar_node])