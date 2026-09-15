from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to the static_tf package
    static_tf_package_path = get_package_share_directory('static_tf')
    static_tf_launch_path = os.path.join(static_tf_package_path, 'launch', 'static_tf_launch.py')

    # Delay between each included launch file so startup is more stable.
    delay_seconds = 3.0

    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_robot_driver'), 'launch', 'ur_control.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur10',
            'robot_ip': '10.0.0.89',
            'launch_rviz': 'true'
        }.items()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur10_tf_gripper_moveit_config'), 'launch', 'demo.launch.py')
        )
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'pointcloud.enable': 'true'
        }.items()
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(static_tf_launch_path)
    )

    return LaunchDescription([
        # Launch UR robot driver immediately
        ur_robot_driver_launch,

        # Launch UR + AP gripper MoveIt configuration after a short delay
        TimerAction(
            period=delay_seconds,
            actions=[moveit_launch]
        ),

        # Launch RealSense camera after another delay
        TimerAction(
            period=delay_seconds * 2,
            actions=[realsense_launch]
        ),

        # Launch static TF after the final delay
        TimerAction(
            period=delay_seconds * 3,
            actions=[static_tf_launch]
        )
    ])
