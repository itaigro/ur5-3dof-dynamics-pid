"""
ur5_pid_node — independent-joint PID with optional gravity feedforward,
linear setpoint ramp, and velocity feedforward.

Control law (set-point regulation, use_ramp=False)
--------------------------------------------------
    e        = q_target - q
    tau      = Kp*e + Ki*integral(e dt) + Kd*(-qd_filtered) + G(q)

Control law (trajectory tracking, use_ramp=True)
------------------------------------------------
    q_d(t)   = q_start + (t/T) * (q_target - q_start)   for 0 <= t <= T
               q_target                                  for t > T
    qd_d(t)  = (q_target - q_start) / T                  for 0 <= t <= T
               0                                         for t > T

    e        = q_d(t) - q
    ed       = qd_d(t) - qd_filtered          (velocity feedforward)
    tau      = Kp*e + Ki*integral(e dt) + Kd*ed + G(q)

The ramp begins when physics is detected to be running (||qd|| exceeds
qd_start_threshold). Until then, q_d = q_start (frozen at initial pose)
and qd_d = 0, so the commanded torque equals G(q_start) — exactly the
hold-still torque, no transient at unpause.

Parameters
----------
q_target            : list[float]  final joint angles [q1,q2,q3] (rad)
kp, ki, kd          : list[float]  PID gains per joint
gravity_ff          : bool         add G(q) feedforward (default True)
integral_limit      : float        anti-windup clamp per component
torque_limit        : float        per-joint torque saturation (N·m)
vel_filter          : float        EMA coefficient alpha for qd low-pass
use_ramp            : bool         enable linear setpoint ramp (default True)
ramp_duration       : float        T in seconds (default 3.0)
qd_start_threshold  : float        ||qd|| threshold to detect unpause (rad/s)
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

_DEFAULT_Q_TARGET = [math.pi / 2, -math.pi / 4, -math.pi / 2]   # side-reach pose
_DEFAULT_KP = [20.0, 60.0, 20.0]
_DEFAULT_KI = [ 1.0,  3.0,  1.0]
_DEFAULT_KD = [ 4.0, 12.0,  4.0]


class PIDNode(Node):
    def __init__(self) -> None:
        super().__init__("ur5_pid_node")

        # ── Parameter declarations ──────────────────────────────────
        self.declare_parameter("q_target",           _DEFAULT_Q_TARGET)
        self.declare_parameter("kp",                 _DEFAULT_KP)
        self.declare_parameter("ki",                 _DEFAULT_KI)
        self.declare_parameter("kd",                 _DEFAULT_KD)
        self.declare_parameter("gravity_ff",         True)
        self.declare_parameter("integral_limit",     30.0)
        self.declare_parameter("torque_limit",       150.0)
        self.declare_parameter("vel_filter",         0.2)
        self.declare_parameter("use_ramp",           True)
        self.declare_parameter("ramp_duration",      3.0)
        self.declare_parameter("qd_start_threshold", 0.01)

        self._q_target    = np.array(self.get_parameter("q_target").value, dtype=float)
        self._kp          = np.array(self.get_parameter("kp").value, dtype=float)
        self._ki          = np.array(self.get_parameter("ki").value, dtype=float)
        self._kd          = np.array(self.get_parameter("kd").value, dtype=float)
        self._gravity_ff  = bool(self.get_parameter("gravity_ff").value)
        self._int_limit   = float(self.get_parameter("integral_limit").value)
        self._tau_limit   = float(self.get_parameter("torque_limit").value)
        self._vel_alpha   = float(self.get_parameter("vel_filter").value)
        self._use_ramp    = bool(self.get_parameter("use_ramp").value)
        self._ramp_T      = float(self.get_parameter("ramp_duration").value)
        self._qd_thresh   = float(self.get_parameter("qd_start_threshold").value)

        # ── Runtime state ───────────────────────────────────────────
        self._integral:    np.ndarray = np.zeros(3)
        self._qd_filtered: np.ndarray = np.zeros(3)
        self._t_prev:      Optional[float] = None

        # Ramp state — populated on first state message + first motion detection
        self._q_start:     Optional[np.ndarray] = None   # measured pose at start
        self._t_ramp_zero: Optional[float] = None        # wall time when ramp begins
        self._ramp_done    = False                       # logged when t > T

        # ── Gravity feedforward ─────────────────────────────────────
        if self._gravity_ff:
            from ur5_3dof.dynamics import gravity_fast
            from ur5_3dof.parameters import default_params
            p = default_params()
            self._G = lambda q: gravity_fast(q, p)
            self.get_logger().info("Gravity feedforward active (analytical).")
        else:
            self._G = lambda q: np.zeros(3)
            self.get_logger().info("Gravity feedforward disabled — pure PID.")

        # ── ROS interface ───────────────────────────────────────────
        self._pub = self.create_publisher(JointState, "torque_command", 10)
        self._sub = self.create_subscription(
            JointState, "joint_states", self._state_cb, 10)

        mode = (f"trajectory tracking, ramp T={self._ramp_T:.1f}s"
                if self._use_ramp else "set-point regulation (step input)")
        self.get_logger().info(
            f"PID ready in {mode} mode | "
            f"target = {np.round(self._q_target, 3).tolist()}")

    # ──────────────────────────────────────────────────────────────────
    def _compute_setpoint(self, t_now: float
                          ) -> tuple[np.ndarray, np.ndarray]:
        """Return (q_d, qd_d) at the current wall-clock time."""
        if not self._use_ramp:
            return self._q_target, np.zeros(3)

        if self._q_start is None or self._t_ramp_zero is None:
            # Physics hasn't started yet; hold at the measured start pose.
            return (self._q_start if self._q_start is not None
                    else self._q_target), np.zeros(3)

        tau_ramp = t_now - self._t_ramp_zero
        if tau_ramp <= 0.0:
            return self._q_start, np.zeros(3)
        if tau_ramp >= self._ramp_T:
            return self._q_target, np.zeros(3)

        # Linear interpolation: q_d = q_start + (t/T)*(q_target - q_start)
        s    = tau_ramp / self._ramp_T
        q_d  = self._q_start + s * (self._q_target - self._q_start)
        qd_d = (self._q_target - self._q_start) / self._ramp_T
        return q_d, qd_d

    # ──────────────────────────────────────────────────────────────────
    def _state_cb(self, msg: JointState) -> None:
        name_idx = {n: i for i, n in enumerate(msg.name)}
        try:
            q  = np.array([msg.position[name_idx[n]] for n in _JOINT_NAMES[:3]], dtype=float)
            qd = np.array([msg.velocity[name_idx[n]] for n in _JOINT_NAMES[:3]], dtype=float)
        except (KeyError, IndexError):
            return

        # Wall clock — Gazebo's joint_state_broadcaster sometimes stamps t=0
        t = self.get_clock().now().nanoseconds * 1e-9

        # Low-pass filter on velocity to suppress noise the D-term would amplify
        a = self._vel_alpha
        self._qd_filtered = a * qd + (1.0 - a) * self._qd_filtered

        # ── Ramp bookkeeping ───────────────────────────────────────
        # First message: store starting pose
        if self._q_start is None:
            self._q_start = q.copy()
            self.get_logger().info(
                f"Recorded q_start = {np.round(self._q_start, 4).tolist()}; "
                f"awaiting physics start (||qd|| > {self._qd_thresh}).")

        # Detect physics start: first sample where ||qd|| exceeds threshold
        if self._use_ramp and self._t_ramp_zero is None:
            if np.linalg.norm(qd) > self._qd_thresh:
                self._t_ramp_zero = t
                self.get_logger().info(
                    f"Physics detected (||qd|| = {np.linalg.norm(qd):.3f}); "
                    f"starting {self._ramp_T:.1f}s ramp to "
                    f"{np.round(self._q_target, 3).tolist()}.")

        # Log when ramp completes
        if (self._use_ramp and self._t_ramp_zero is not None
                and not self._ramp_done
                and (t - self._t_ramp_zero) >= self._ramp_T):
            self._ramp_done = True
            self.get_logger().info(
                f"Ramp complete at t = {t - self._t_ramp_zero:.3f}s; "
                "now regulating to target.")

        # ── Setpoint at this instant ───────────────────────────────
        q_d, qd_d = self._compute_setpoint(t)

        # ── PID with velocity feedforward ──────────────────────────
        e  = q_d - q
        ed = qd_d - self._qd_filtered

        dt = 0.0 if self._t_prev is None else max(t - self._t_prev, 0.0)
        self._t_prev = t

        # Conditional integration: only when error is small enough
        if np.linalg.norm(e) < 1.5:
            self._integral += e * dt
            np.clip(self._integral, -self._int_limit, self._int_limit,
                    out=self._integral)

        tau = (self._kp * e
               + self._ki * self._integral
               + self._kd * ed          # velocity FF: was -kd*qd_filtered
               + self._G(q))
        np.clip(tau, -self._tau_limit, self._tau_limit, out=tau)

        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name   = _JOINT_NAMES
        out.effort = [float(tau[0]), float(tau[1]), float(tau[2]),
                      0.0, 0.0, 0.0]
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
