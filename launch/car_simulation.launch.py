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
        name='GZ_SIM_RESOURCE_PATH', 
        value=PathJoinSubstitution([pkg_share, "worlds"])
    )
    
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", world],
        output="screen"
    )
    
    lidar_node = Node(
        package="lidar_sim_pkg",
        executable="lidar_node",
        name="lidar_node",
        output="screen",
    )
    
    gz_lidar_topic = '/lidar/points'
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[f'{gz_lidar_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        remappings=[(gz_lidar_topic, "/livox/lidar")],
        output='screen',
    )

    gz_imu_topic = '/imu'
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[f'{gz_imu_topic}@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        remappings=[(gz_imu_topic, "/livox/imu")],
        output='screen',
    )

    gz_odom_topic = '/model/vehicle_blue/odometry'
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[f'{gz_odom_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[(gz_odom_topic, "/odom")],
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
        set_res_path, 
        gz_sim, 
        lidar_node,
        lidar_bridge,
        imu_bridge,
        odom_bridge,
        # tf2_ros,
    ])