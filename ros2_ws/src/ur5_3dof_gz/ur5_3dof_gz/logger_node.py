"""
ur5_logger_node — records /joint_states and saves CSV + plots on shutdown.

In the Gazebo path, joint_state_broadcaster publishes position, velocity,
and effort (the commanded torque) in the same /joint_states message,
so this node needs only one subscription.

On SIGINT (Ctrl-C), saves:
  <save_dir>/<prefix>_trajectory.csv
  <save_dir>/<prefix>_joints[_torques].png   (matplotlib)
  <save_dir>/<prefix>_stick_figure.png

Parameters
----------
save_dir   : string       directory to write results (absolute or relative to CWD)
prefix     : string       filename prefix
q_target   : list[float]  if has_target=True, dashed target lines on the angle plot
has_target : bool         True if q_target should be shown
plot       : bool         save matplotlib plots (default True)
"""
from __future__ import annotations

import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

_ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"]


class LoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("ur5_logger_node")

        self.declare_parameter("save_dir", "results")
        self.declare_parameter("prefix",   "ros_sim")
        self.declare_parameter("q_target", [0.0, 0.0, 0.0])
        self.declare_parameter("has_target", False)
        self.declare_parameter("plot",     True)

        self._save_dir  = Path(self.get_parameter("save_dir").value)
        self._save_dir.mkdir(parents=True, exist_ok=True)
        self._prefix    = self.get_parameter("prefix").value
        self._do_plot   = bool(self.get_parameter("plot").value)
        has_target      = bool(self.get_parameter("has_target").value)
        self._q_target  = (
            np.array(self.get_parameter("q_target").value, dtype=float)
            if has_target else None
        )

        self._t:   list[float]       = []
        self._q:   list[list[float]] = []
        self._qd:  list[list[float]] = []
        self._tau: list[list[float]] = []
        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            JointState, "joint_states", self._state_cb, 100)

        self.get_logger().info(
            f"Logging to {self._save_dir}/{self._prefix}_*")

    # ------------------------------------------------------------------
    def _state_cb(self, msg: JointState) -> None:
        name_idx = {n: i for i, n in enumerate(msg.name)}
        try:
            idx = [name_idx[n] for n in _ARM_JOINTS]
        except KeyError:
            return
        # Wall clock — Gazebo's joint_state_broadcaster stamps messages at t=0
        t = self.get_clock().now().nanoseconds * 1e-9
        q   = [float(msg.position[i]) for i in idx]
        qd  = [float(msg.velocity[i]) for i in idx] if len(msg.velocity) > max(idx) else [0.0]*3
        tau = [float(msg.effort[i])   for i in idx] if len(msg.effort)   > max(idx) else [0.0]*3
        with self._lock:
            self._t.append(t)
            self._q.append(q)
            self._qd.append(qd)
            self._tau.append(tau)

    # ------------------------------------------------------------------
    def save(self) -> None:
        with self._lock:
            if not self._t:
                self.get_logger().warn("No data recorded — nothing to save.")
                return
            t   = np.array(self._t)
            q   = np.array(self._q)
            qd  = np.array(self._qd)
            tau = np.array(self._tau)

        t = t - t[0]   # normalise to start at 0

        csv_path = self._save_dir / f"{self._prefix}_trajectory.csv"
        header = "t,q1,q2,q3,qd1,qd2,qd3,tau1,tau2,tau3"
        np.savetxt(csv_path, np.column_stack([t, q, qd, tau]),
                   delimiter=",", header=header, comments="")
        self.get_logger().info(f"Saved → {csv_path}")

        if not self._do_plot:
            return

        import matplotlib
        matplotlib.use("Agg")
        from ur5_3dof.visualization import (
            plot_joint_trajectories,
            plot_with_torques,
            stick_figure_snapshots,
        )

        if self._q_target is not None:
            plot_path = self._save_dir / f"{self._prefix}_joints_torques.png"
            plot_with_torques(
                t, q, qd, tau,
                q_target=self._q_target,
                title="PID set-point regulation (Gazebo)",
                save_path=plot_path,
            )
        else:
            plot_path = self._save_dir / f"{self._prefix}_joints.png"
            plot_joint_trajectories(
                t, q, qd,
                title=r"Uncontrolled motion under gravity ($\tau = 0$) — Gazebo",
                save_path=plot_path,
            )
        self.get_logger().info(f"Saved → {plot_path}")

        stick_path = self._save_dir / f"{self._prefix}_stick_figure.png"
        stick_figure_snapshots(
            t, q,
            title=f"Stick-figure — {self._prefix}",
            save_path=stick_path,
        )
        self.get_logger().info(f"Saved → {stick_path}")


def main() -> None:
    rclpy.init()
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
