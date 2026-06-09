"""
Tuning iteration v1 — aggressive gains, pure set-point regulation.

  kp = [20, 60, 20]
  ki = [ 1,  3,  1]
  kd = [ 4, 12,  4]
  use_ramp = False    (step input from q0 directly to q_target)

This launch starts Gazebo PAUSED and only unpauses once everything is
loaded. The PID is given a step input — the target jumps from "hold q0"
to q_target the moment physics begins.

Launch sequence:
  t = 0  s  Gazebo starts paused; robot_state_publisher starts
  t = 3  s  Robot spawns (frozen in initial pose because physics is paused)
  t = 10 s  Controllers load
  t = 12 s  Bridge starts publishing zero torque
  t = 13 s  PID + logger start
  t = 14 s  Gazebo unpaused — physics begins, PID jumps to commanding q_target

Output prefix: gazebo_pid_v1_aggressive_*

Usage:
  ros2 launch ur5_3dof_gz gazebo_pid_v1_aggressive.launch.py
"""
from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, ExecuteProcess,
    IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_Q_TARGET = [math.pi / 2, -math.pi / 4, -math.pi / 2]   # pickup pose
_PREFIX   = "gazebo_pid_v1_aggressive"


def generate_launch_description() -> LaunchDescription:
    gz_share   = get_package_share_directory("ros_gz_sim")
    ur5_gz_pkg = get_package_share_directory("ur5_3dof_gz")

    urdf_xacro = os.path.join(ur5_gz_pkg, "urdf", "ur5_3dof_gz.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", urdf_xacro]), value_type=str)

    set_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", "/opt/ros/jazzy/share",
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "empty.sdf", "pause": "true"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=["-name", "ur5", "-topic", "/robot_description"],
            )
        ],
    )

    controllers_yaml = os.path.join(ur5_gz_pkg, "config", "controllers.yaml")

    def spawner(name: str) -> Node:
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--param-file", controllers_yaml],
            output="screen",
        )

    delayed_controllers = TimerAction(
        period=10.0,
        actions=[
            spawner("joint_state_broadcaster"),
            spawner("arm_effort_controller"),
            spawner("wrist_position_controller"),
        ],
    )

    bridge_node = Node(
        package="ur5_3dof_gz",
        executable="torque_bridge",
        name="torque_bridge",
        output="screen",
    )

    delayed_bridge = TimerAction(period=12.0, actions=[bridge_node])

    pid_node = Node(
        package="ur5_3dof_gz",
        executable="ur5_pid_node",
        name="ur5_pid_node",
        output="screen",
        parameters=[{
            "q_target":       _Q_TARGET,
            "kp":             [20.0, 60.0, 20.0],   # v1: aggressive
            "ki":             [ 1.0,  3.0,  1.0],
            "kd":             [ 4.0, 12.0,  4.0],   # v1: aggressive
            "gravity_ff":     True,
            "integral_limit": 30.0,
            "torque_limit":   150.0,
            "vel_filter":     0.2,
            "use_ramp":       False,                # v1: step input
        }],
    )

    results_dir = os.path.join(
        os.path.expanduser("~"), "ur5-3dof-dynamics-pid", "results")

    logger_node = Node(
        package="ur5_3dof_gz",
        executable="ur5_logger_node",
        name="ur5_logger_node",
        output="screen",
        parameters=[{
            "save_dir":   results_dir,
            "prefix":     _PREFIX,
            "q_target":   _Q_TARGET,
            "has_target": True,
            "plot":       True,
        }],
    )

    delayed_pid_logger = TimerAction(period=13.0, actions=[pid_node, logger_node])

    unpause = TimerAction(
        period=14.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "gz", "service", "-s", "/world/empty/control",
                    "--reqtype", "gz.msgs.WorldControl",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "3000",
                    "--req", "pause: false",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        set_resource_path,
        gz, rsp, spawn,
        delayed_controllers,
        delayed_bridge,
        delayed_pid_logger,
        unpause,
    ])
