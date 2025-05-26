# realsense_manager.py

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


class RealSenseCameraLauncher:
    def __init__(self,
                 camera_name='camera',
                 enable_color=True,
                 enable_depth=True,
                 enable_imu=False,
                 serial_no=None):
        self.camera_name = camera_name
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.enable_imu = enable_imu
        self.serial_no = serial_no

    def get_launch_description(self):
        """LaunchDescription을 반환하여 카메라 노드를 시작"""
        parameters = {
            'enable_color': self.enable_color,
            'enable_depth': self.enable_depth,
            'enable_gyro': self.enable_imu,
            'enable_accel': self.enable_imu,
            
            'align_depth': True,                   # 정렬 활성화
            'pointcloud.enable': True,             # 정렬된 depth가 생성되기 위한 조건
            'publish_odom_tf': False,

            'depth_module.profile': '640x480x30',  # 깊이 프로필
            'rgb_camera.profile': '640x480x30',    # 컬러 프로필
            'depth_fps': 30,
            'color_fps': 30
        }


        if self.serial_no:
            parameters['serial_no'] = self.serial_no

        camera_node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=self.camera_name,
            output='screen',
            parameters=[parameters],
            remappings=[
                ('/depth/image_rect_raw', f'/{self.camera_name}/depth/image_rect_raw'),
                ('/color/image_raw', f'/{self.camera_name}/color/image_raw'),
            ]
        )

        return LaunchDescription([
            TimerAction(period=1.0, actions=[camera_node])  # 1초 지연 후 실행
        ])

