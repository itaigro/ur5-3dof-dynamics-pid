"""
Forward kinematics of the 3-DOF UR5 model.

Uses the standard Denavit-Hartenberg convention (the same one shipped in
Universal Robots' official kinematic documentation), restricted to the
first three rows of the table.
"""

from __future__ import annotations

import numpy as np

from .parameters import UR5Params


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Standard DH transform: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d     ],
        [0.0, 0.0,      0.0,     1.0   ],
    ])


def forward_kinematics(q: np.ndarray, p: UR5Params) -> dict:
    """Return positions of base, shoulder, elbow, and wrist origin in world."""
    q1, q2, q3 = q

    T01 = dh_transform(a=0.0,  alpha=np.pi / 2, d=p.d1, theta=q1)
    T12 = dh_transform(a=p.a2, alpha=0.0,        d=0.0,  theta=q2)
    T23 = dh_transform(a=p.a3, alpha=0.0,        d=0.0,  theta=q3)

    T02 = T01 @ T12
    T03 = T02 @ T23

    return {
        "base":     np.zeros(3),
        "shoulder": T01[:3, 3],
        "elbow":    T02[:3, 3],
        "wrist":    T03[:3, 3],
        "T01": T01,
        "T02": T02,
        "T03": T03,
    }


def com_positions(q: np.ndarray, p: UR5Params) -> dict:
    """World-frame positions of each link's centre of mass."""
    fk = forward_kinematics(q, p)
    T01, T02, T03 = fk["T01"], fk["T02"], fk["T03"]
    com1_h = T01 @ np.append(p.com1, 1.0)
    com2_h = T02 @ np.append(p.com2, 1.0)
    com3_h = T03 @ np.append(p.com3, 1.0)
    return {
        "com1": com1_h[:3],
        "com2": com2_h[:3],
        "com3": com3_h[:3],
    }


if __name__ == "__main__":
    from .parameters import default_params

    p = default_params()
    print("Stick-figure points at q = [0, 0, 0]:")
    for name, pt in forward_kinematics(np.zeros(3), p).items():
        if name.startswith("T"):
            continue
        print(f"  {name:9s} = {pt}")

    print("\nStick-figure points at q = [0, -pi/2, 0] (arm vertical):")
    for name, pt in forward_kinematics(np.array([0.0, -np.pi / 2, 0.0]), p).items():
        if name.startswith("T"):
            continue
        print(f"  {name:9s} = {pt}")
