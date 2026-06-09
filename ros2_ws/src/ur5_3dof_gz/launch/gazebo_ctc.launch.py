"""
Computed-torque (feedback-linearization) control.

Control law:
  tau = M(q) [Kp*(q_d - q) + Kd*(qd_d - qd)] + C(q,qd)*qd + G(q)

The dynamics are linearized and decoupled by the inner loop (M, C, G).
The outer loop is independent-joint PID on three uncoupled second-order
systems, with critical damping at omega_n = 4 rad/s:
  Kp = 16, Kd = 8 (same for all three joints)

Step input — this is the whole point of computed torque.
Expected: clean monotonic motion, settling in ~1.5 s, no overshoot.

Usage:
  # Default target (pickup pose)
  ros2 launch ur5_3dof_gz gazebo_ctc.launch.py

  # Custom target — joint angles in radians
  ros2 launch ur5_3dof_gz gazebo_ctc.launch.py \\
       q_target:="[1.0, -0.5, -1.0]"

  # Custom target with a custom output-file prefix
  ros2 launch ur5_3dof_gz gazebo_ctc.launch.py \\
       q_target:="[1.0, -0.5, -1.0]" \\
       prefix:="gazebo_ctc_side_reach"

The dashed reference lines in the saved plot reflect the chosen target.
"""
from __future__ import annotations

import ast
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_DEFAULT_Q_TARGET_STR = "[1.5707963267948966, -0.7853981633974483, -1.5707963267948966]"
_DEFAULT_PREFIX       = "gazebo_ctc"


def _parse_q_target(raw: str) -> list[float]:
    """Parse '[a, b, c]' (or 'a, b, c') into [float, float, float].

    Raises ValueError with a friendly message if the input is malformed.
    """
    try:
        value = ast.literal_eval(raw.strip())
    except (ValueError, SyntaxError) as exc:
        raise ValueError(
            f"q_target must look like '[q1, q2, q3]' (Python list syntax). "
            f"Got: {raw!r}. Parser error: {exc}"
        )

    if isinstance(value, (int, float)):
        raise ValueError(
            f"q_target must be a list of 3 numbers, got a single number {value!r}.")
    try:
        items = list(value)
    except TypeError:
        raise ValueError(
            f"q_target must be a list/tuple of 3 numbers, got {type(value).__name__}: {value!r}.")

    if len(items) != 3:
        raise ValueError(
            f"q_target must contain exactly 3 numbers (q1, q2, q3), got {len(items)}: {items!r}.")

    out = []
    for i, x in enumerate(items, start=1):
        if not isinstance(x, (int, float)):
            raise ValueError(
                f"q_target[{i-1}] (q{i}) is not a number: {x!r} (type {type(x).__name__}).")
        out.append(float(x))

    # Conservative reachability check. The UR5 joint limits are wider than
    # these, but values outside this range typically push the arm through
    # the ground plane or into self-collision in Gazebo.
    _SAFE_RANGES = [
        (-6.28,  6.28),    # q1 (base): essentially unrestricted
        (-3.14, -0.10),    # q2 (shoulder): keep arm above horizontal
        (-2.50,  2.50),    # q3 (elbow): avoid extreme folds
    ]
    _JOINT_NAMES = ("base", "shoulder", "elbow")
    for i, (val, (lo, hi), name) in enumerate(
            zip(out, _SAFE_RANGES, _JOINT_NAMES), start=1):
        if val < lo or val > hi:
            raise ValueError(
                f"q_target[{i-1}] (q{i}, {name}) = {val:.3f} is outside the "
                f"safe range [{lo:.3f}, {hi:.3f}]. Targets outside this range "
                f"typically cause the arm to collide with the ground or itself "
                f"in Gazebo. If you really mean to try this value, edit the "
                f"_SAFE_RANGES table in gazebo_ctc.launch.py."
            )
    return out


def _build_actions(context, *args, **kwargs):
    """Resolve launch substitutions, validate target, then build the node graph."""
    q_target_raw = LaunchConfiguration("q_target").perform(context)
    prefix       = LaunchConfiguration("prefix").perform(context)

    try:
        q_target = _parse_q_target(q_target_raw)
    except ValueError as exc:
        print(f"\n[gazebo_ctc.launch.py] ERROR: {exc}\n")
        raise

    print(f"\n[gazebo_ctc.launch.py] Using q_target = {q_target}")
    print(f"[gazebo_ctc.launch.py] Output prefix = {prefix!r}\n")

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

    ctc_node = Node(
        package="ur5_3dof_gz",
        executable="ur5_ctc_node",
        name="ur5_ctc_node",
        output="screen",
        parameters=[{
            "q_target":     q_target,
            "kp":           [16.0, 16.0, 16.0],
            "kd":           [ 8.0,  8.0,  8.0],
            "torque_limit": 150.0,
            "vel_filter":   0.5,
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
            "prefix":     prefix,
            "q_target":   q_target,
            "has_target": True,
            "plot":       True,
        }],
    )

    delayed_pid_logger = TimerAction(period=13.0, actions=[ctc_node, logger_node])

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

    return [
        set_resource_path,
        gz, rsp, spawn,
        delayed_controllers,
        delayed_bridge,
        delayed_pid_logger,
        unpause,
    ]


def generate_launch_description() -> LaunchDescription:
    q_target_arg = DeclareLaunchArgument(
        "q_target",
        default_value=_DEFAULT_Q_TARGET_STR,
        description="Target joint angles as a Python list, e.g. '[1.0, -0.5, -1.0]' (rad).",
    )
    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value=_DEFAULT_PREFIX,
        description="Filename prefix for the saved CSV and plots in results/.",
    )

    return LaunchDescription([
        q_target_arg,
        prefix_arg,
        OpaqueFunction(function=_build_actions),
    ])
