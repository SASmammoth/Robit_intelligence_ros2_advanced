from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "urdf", "eclipse.xacro"]
            ),
        ]
    )
    robot_description = ParameterValue(urdf_file, value_type=str)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "config", "eclipse.rviz"]
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[ # AI를 이용하여 추가함
                    {
                        "source_list": ["/ik/joint_states"],  # IK 노드 값을 받아서 합침
                        "rate": 30,  # 발행 주기 (기본 10 Hz)
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
            ),
        ]
    )
