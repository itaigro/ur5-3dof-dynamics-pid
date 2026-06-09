"""
ur5_ctc_node — computed-torque (feedback-linearization) controller.

Control law:
    tau = M(q) [q̈_d + Kd·(q̇_d − q̇) + Kp·(q_d − q)] + C(q, q̇)·q̇ + G(q)

Substituting into the dynamics M·q̈ + C·q̇ + G = tau:
    M·q̈ = M·[q̈_d + Kd·(q̇_d − q̇) + Kp·(q_d − q)]
        q̈ = q̈_d + Kd·(q̇_d − q̇) + Kp·(q_d − q)

Letting e = q_d − q, the closed-loop dynamics on the error become:
    ë + Kd·ė + Kp·e = 0

— three independent decoupled second-order systems. Choosing Kp = ω_n²
and Kd = 2·ω_n gives critical damping (ζ=1) at natural frequency ω_n.

Default ω_n = 4 rad/s → Kp = 16, Kd = 8. Settling time ≈ 1.5 s.

Reference: Spong, Hutchinson, Vidyasagar, "Robot Modeling and Control",
Chapter 8 (Inverse Dynamics / Computed Torque).
Course syllabus week 11: "Feedback linearization".

Parameters
----------
q_target            : list[float]  final joint angles [q1,q2,q3] (rad)
kp                  : list[float]  Kp gains (default [16, 16, 16] = ω_n²)
kd                  : list[float]  Kd gains (default [ 8,  8,  8] = 2ω_n)
torque_limit        : float        per-joint torque saturation (N·m)
vel_filter          : float        EMA coefficient α for qd low-pass
qd_start_threshold  : float        ||qd|| threshold to detect physics start
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

_DEFAULT_Q_TARGET = [math.pi / 2, -math.pi / 4, -math.pi / 2]
# ω_n = 4 rad/s → Kp = 16, Kd = 8 (critical damping)
_DEFAULT_KP = [16.0, 16.0, 16.0]
_DEFAULT_KD = [ 8.0,  8.0,  8.0]


class CTCNode(Node):
    def __init__(self) -> None:
        super().__init__("ur5_ctc_node")

        self.declare_parameter("q_target",           _DEFAULT_Q_TARGET)
        self.declare_parameter("kp",                 _DEFAULT_KP)
        self.declare_parameter("kd",                 _DEFAULT_KD)
        self.declare_parameter("torque_limit",       150.0)
        self.declare_parameter("vel_filter",         0.5)
        self.declare_parameter("qd_start_threshold", 0.01)

        self._q_target  = np.array(self.get_parameter("q_target").value, dtype=float)
        self._Kp        = np.array(self.get_parameter("kp").value, dtype=float)
        self._Kd        = np.array(self.get_parameter("kd").value, dtype=float)
        self._tau_limit = float(self.get_parameter("torque_limit").value)
        self._vel_alpha = float(self.get_parameter("vel_filter").value)
        self._qd_thresh = float(self.get_parameter("qd_start_threshold").value)

        self._qd_filtered: np.ndarray = np.zeros(3)
        self._q_start:     Optional[np.ndarray] = None
        self._physics_started = False

        # Load fast dynamics functions
        from ur5_3dof.dynamics import gravity_fast, mass_inertia_fast, coriolis_fast
        from ur5_3dof.parameters import default_params
        p = default_params()
        self._G = lambda q: gravity_fast(q, p)
        self._M = lambda q: mass_inertia_fast(q, p)
        self._C = lambda q, qd: coriolis_fast(q, qd, p)

        self._pub = self.create_publisher(JointState, "torque_command", 10)
        self._sub = self.create_subscription(
            JointState, "joint_states", self._state_cb, 10)

        self.get_logger().info(
            f"CTC ready (computed-torque control) | "
            f"target = {np.round(self._q_target, 3).tolist()} | "
            f"Kp = {self._Kp.tolist()}, Kd = {self._Kd.tolist()}")

    # ──────────────────────────────────────────────────────────────────
    def _state_cb(self, msg: JointState) -> None:
        name_idx = {n: i for i, n in enumerate(msg.name)}
        try:
            q  = np.array([msg.position[name_idx[n]] for n in _JOINT_NAMES[:3]], dtype=float)
            qd = np.array([msg.velocity[name_idx[n]] for n in _JOINT_NAMES[:3]], dtype=float)
        except (KeyError, IndexError):
            return

        # Low-pass filter on velocity
        a = self._vel_alpha
        self._qd_filtered = a * qd + (1.0 - a) * self._qd_filtered

        # Track starting pose for logging
        if self._q_start is None:
            self._q_start = q.copy()
            self.get_logger().info(
                f"Recorded q_start = {np.round(self._q_start, 4).tolist()}")

        # Note when physics begins (for logging only)
        if not self._physics_started and np.linalg.norm(qd) > self._qd_thresh:
            self._physics_started = True
            self.get_logger().info(
                f"Physics started; commanding step from q_start to q_target.")

        # ── Computed-torque control law ────────────────────────────
        # Step input: q_d = q_target, qd_d = 0, qdd_d = 0
        e  = self._q_target - q
        ed = -self._qd_filtered            # qd_d = 0

        # Outer-loop "synthetic" acceleration command
        # qdd_command = qdd_d + Kp*e + Kd*ed
        #             = 0     + Kp*e + Kd*(-qd)
        qdd_command = self._Kp * e + self._Kd * ed

        # Inner-loop: linearize via M, C, G
        # tau = M(q) * qdd_command + C(q, qd) * qd + G(q)
        M = self._M(q)
        C = self._C(q, qd)
        tau = M @ qdd_command + C @ qd + self._G(q)

        np.clip(tau, -self._tau_limit, self._tau_limit, out=tau)

        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name   = _JOINT_NAMES
        out.effort = [float(tau[0]), float(tau[1]), float(tau[2]),
                      0.0, 0.0, 0.0]
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = CTCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
