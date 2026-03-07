from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    imu_node = Node(
        package="imu_plane",
        executable="imu_node.py",
        name="imu_node",
        output="screen"
    )

    tf_node = Node(
        package="imu_plane",
        executable="tf_broadcaster_imu.py",
        name="imu_tf_broadcaster",
        output="screen"
    )

    return LaunchDescription([
        imu_node,
        tf_node
    ])