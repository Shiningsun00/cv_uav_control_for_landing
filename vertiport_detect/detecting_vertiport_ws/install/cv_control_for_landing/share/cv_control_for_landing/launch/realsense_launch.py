from launch import LaunchDescription
from cv_control_for_landing.RealSenseCameraLauncher import RealSenseCameraLauncher

def generate_launch_description():
    launcher = RealSenseCameraLauncher(
        camera_name='realsense_front',
        enable_color=True,
        enable_depth=True,
        enable_imu=True
    )
    return launcher.get_launch_description()

