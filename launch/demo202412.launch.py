import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ros_tcp_endpoint_dir = get_package_share_directory('ros_tcp_endpoint')
    ic120_unity_dir = get_package_share_directory('ic120_unity')
    zx200_unity_dir = get_package_share_directory('zx200_unity')
    tms_ts_launch_dir = get_package_share_directory('tms_ts_launch')
    simizu_project_dir = get_package_share_directory('simizu_project')
    tms_if_for_opera_dir = get_package_share_directory('tms_if_for_opera')
    ros_tcp_endpoint_launch_file = os.path.join(ros_tcp_endpoint_dir, 'launch', 'endpoint.py')
    ic120_standby_ekf_launch_file = os.path.join(ic120_unity_dir, 'launch', 'ic120_standby_ekf.launch.py')
    zx200_standby_launch_file = os.path.join(zx200_unity_dir, 'launch', 'zx200_standby.launch.py')
    tms_ts_demo202412_launch_file = os.path.join(tms_ts_launch_dir, 'launch', 'tms_ts_demo202412.launch.py')
    tms_if_for_opera_launch_file = os.path.join(tms_if_for_opera_dir, 'launch', 'tms_if_for_opera.launch.py')
    sample_rviz_file = os.path.join(simizu_project_dir, 'rviz2', 'sample.rviz')
    return LaunchDescription([

        DeclareLaunchArgument('prefix_ic120_1', default_value='ic120'),
        DeclareLaunchArgument('use_rviz_ic120', default_value='false'),
        DeclareLaunchArgument('use_rviz_zx200', default_value='false'),
        DeclareLaunchArgument('task_id1', default_value='8'),
        DeclareLaunchArgument('task_id2', default_value='9'),

        # ros_tcp_endpoint
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_tcp_endpoint_launch_file),
        ),
        
        # IC120用のナビゲーション用
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ic120_standby_ekf_launch_file),
            launch_arguments={
                'common_prefix': LaunchConfiguration('prefix_ic120_1'),
                'use_rviz' : LaunchConfiguration('use_rviz_ic120'),
            }.items(),
        ),

        # Zx200のmap -> base_link間のtfを吐くノード
        # PhysX上のZX200はシミュレータ上のbase_linkの位置の真値を吐いているため、その情報をmap -> base_link間のtfとして吐いています
        # 実機ではmap -> base_link間は接続できていません。
        Node(
            package='simizu_project',
            executable='tf_broadcaster_map_to_baselink',
            name='tf_broadcaster_map_to_baselink',
        ),

        # ZX200のマニピュレーション用
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zx200_standby_launch_file),
            launch_arguments={'use_rviz': launch.substitutions.LaunchConfiguration('use_rviz_zx200')}.items()
        ),

        # ROS2-TMS for Construction用
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tms_ts_demo202412_launch_file),
            launch_arguments={
                'task_id1': launch.substitutions.LaunchConfiguration('task_id1'),
                'task_id2': launch.substitutions.LaunchConfiguration('task_id2')
                }.items(),
        ),

        # tms_if_for_opera用 (ROS2-TMS for ConstructionとOPERA接続用)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tms_if_for_opera_launch_file),
        ),

        # マップ表示用
        # Node(
        #     package='simizu_project',
        #     executable='operasim_physx_default_map_publisher',
        #     name='operasim_physx_default_map_publisher',
        #     namespace = 'map_coodinater',
        #     parameters=[{'image_file': 'simizu_project'},
        #                 {'origin_x': -49.0},
        #                 {'origin_y': -47.0}]),
        
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[{'use_sim_time': True}],
            arguments=["--display-config", sample_rviz_file]),
    ])