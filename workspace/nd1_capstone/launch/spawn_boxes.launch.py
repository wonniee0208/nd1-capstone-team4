#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  박스 스폰 launch — 실행 중인 TurtleBot4 기본 월드에 박스 배치
#  사용:
#    ros2 launch nd1_capstone spawn_boxes.launch.py world:=<월드명>
#  ※ <월드명>은 실행 중인 Ignition 월드 이름. 확인:
#       ign topic -l | grep -m1 world      (예: /world/warehouse/...)
#    TurtleBot4 ignition 기본 월드는 보통 'warehouse'.
#  ※ ros_gz_sim 의 create 노드가 /world/<world>/create 서비스로 스폰.
#    (구버전 환경이면 package='ros_ign_gazebo' 로 교체)
# ════════════════════════════════════════════════════════════════
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("nd1_capstone")
    sdf = os.path.join(pkg, "models", "box1.sdf")
    world = LaunchConfiguration("world")

    # 기본은 A구역(1.5, 0.5)에 box1 1개. 상 난이도(다중)면 box_B/C 주석 해제.
    spawn_box1 = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-world", world, "-file", sdf,
                   "-name", "box1", "-x", "1.5", "-y", "0.5", "-z", "0.1"])

    # spawn_box_B = Node(
    #     package="ros_gz_sim", executable="create", output="screen",
    #     arguments=["-world", world, "-file", sdf,
    #                "-name", "box_B", "-x", "2.5", "-y", "-1.0", "-z", "0.1"])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="warehouse",
                              description="실행 중인 Ignition 월드 이름 (ign topic -l 로 확인)"),
        spawn_box1,
        # spawn_box_B,
    ])
