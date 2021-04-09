from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
        package="aruco-detector",
        executable="aruco-detector",
        arguments=[
            "/usr/local/share/ORB_SLAM2/Vocabulary/orb_mur.fbow",
            "/usr/local/share/ORB_SLAM2/Config/RealSense-D435i-IRD.yaml",
            "IRD"
        ]
    )
    ])
