"""Plotting and stick-figure helpers for the 3-DOF UR5 simulation."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.figure import Figure

from .kinematics import forward_kinematics
from .parameters import UR5Params, default_params


JOINT_LABELS = (r"$q_1$ (base)", r"$q_2$ (shoulder)", r"$q_3$ (elbow)")


def plot_joint_trajectories(t: np.ndarray, q: np.ndarray, qd: np.ndarray,
                            title: str = "",
                            save_path: Optional[Path] = None) -> Figure:
    """Two-row plot: joint angles (top) and joint velocities (bottom)."""
    fig, axes = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    for i in range(3):
        axes[0].plot(t, q[:, i],  label=JOINT_LABELS[i])
        axes[1].plot(t, qd[:, i], label=JOINT_LABELS[i])
    axes[0].set_ylabel("joint angle [rad]")
    axes[1].set_ylabel(r"joint velocity [rad/s]")
    axes[1].set_xlabel("time [s]")
    if title:
        axes[0].set_title(title)
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=150)
    return fig


def plot_with_torques(t: np.ndarray, q: np.ndarray, qd: np.ndarray,
                      tau: np.ndarray, q_target: Optional[np.ndarray] = None,
                      title: str = "",
                      save_path: Optional[Path] = None) -> Figure:
    """Three-row plot: angles, velocities, torques. Optional target overlay."""
    fig, axes = plt.subplots(3, 1, figsize=(9, 7.5), sharex=True)
    for i in range(3):
        axes[0].plot(t, q[:, i],  label=JOINT_LABELS[i])
        if q_target is not None:
            axes[0].axhline(q_target[i], linestyle="--", alpha=0.5,
                            color=f"C{i}")
        axes[1].plot(t, qd[:, i], label=JOINT_LABELS[i])
        axes[2].plot(t, tau[:, i], label=JOINT_LABELS[i])
    axes[0].set_ylabel("joint angle [rad]")
    axes[1].set_ylabel(r"velocity [rad/s]")
    axes[2].set_ylabel(r"torque $\tau$ [N$\cdot$m]")
    axes[2].set_xlabel("time [s]")
    if title:
        axes[0].set_title(title)
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=150)
    return fig


def _draw_arm(ax, q: np.ndarray, p: UR5Params, color="C0", alpha=1.0,
              label: Optional[str] = None) -> None:
    fk = forward_kinematics(q, p)
    pts = np.stack([fk["base"], fk["shoulder"], fk["elbow"], fk["wrist"]])
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color=color, alpha=alpha,
            linewidth=2, markersize=5, label=label)


def stick_figure_snapshots(t: np.ndarray, q: np.ndarray,
                           p: Optional[UR5Params] = None,
                           n_frames: int = 6,
                           title: str = "",
                           save_path: Optional[Path] = None,
                           ) -> Figure:
    """Overlay n_frames stick-figure snapshots in a single 3D plot."""
    if p is None:
        p = default_params()
    idxs = np.linspace(0, len(t) - 1, n_frames, dtype=int)
    cmap = plt.cm.viridis
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    for k, i in enumerate(idxs):
        c = cmap(k / max(n_frames - 1, 1))
        alpha = 0.35 + 0.65 * k / max(n_frames - 1, 1)
        _draw_arm(ax, q[i], p, color=c, alpha=alpha,
                  label=f"t={t[i]:.2f}s")
    ax.plot([0, 0], [0, 0], [0, p.d1], "k:", alpha=0.5)
    reach = abs(p.a2) + abs(p.a3) + 0.1
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_zlim(0, reach)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    if title:
        ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass
    fig.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=150)
    return fig


def save_trajectory_csv(t: np.ndarray, q: np.ndarray, qd: np.ndarray,
                        tau: np.ndarray, path: Path) -> None:
    """CSV with columns t, q1, q2, q3, qd1, qd2, qd3, tau1, tau2, tau3."""
    data = np.column_stack([t, q, qd, tau])
    header = "t,q1,q2,q3,qd1,qd2,qd3,tau1,tau2,tau3"
    np.savetxt(path, data, delimiter=",", header=header, comments="")
