from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    arm_share = get_package_share_directory("arm")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_share, "model", "robotic_arm.urdf")
    controller_yaml = os.path.join(launch_pack_share, "config", "ros2_controller.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    default_mjcf = os.path.join(arm_share, "model", "scene.xml")

    mjcf_path_arg = DeclareLaunchArgument(
        "mjcf_path",
        default_value=default_mjcf,
        description="Path to the robotic arm MJCF/scene XML used by MuJoCo",
    )

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 together with MuJoCo simulation",
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            {"robot_description": robot_desc},
            controller_yaml,
            {"simulation_frequency": 500.0},
            {"real_time_factor": 1.0},
            {"robot_model_path": LaunchConfiguration("mjcf_path")},
            {"show_gui": True},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    sim_pid_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sim_pid_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_calc = Node(
        package="arm_calc",
        executable="arm_calc",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
    )

    load_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="MuJoCo started, spawning sim_pid_controller"),
                sim_pid_controller,
            ],
        )
    )

    load_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=sim_pid_controller,
            on_exit=[
                LogInfo(msg="sim_pid_controller spawned, spawning arm_controller"),
                arm_controller,
            ],
        )
    )

    return LaunchDescription([
        mjcf_path_arg,
        show_rviz_arg,
        robot_state_pub,
        mujoco,
        load_controller,
        load_arm_controller,
        arm_calc,
        rviz2,
    ])
