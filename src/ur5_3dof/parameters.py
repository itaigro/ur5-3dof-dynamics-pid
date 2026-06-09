"""
Kinematic and inertial parameters of the UR5 manipulator, restricted to
joints 1-3 (base, shoulder, elbow).

We freeze joints 4-6 (wrist 1/2/3) at their zero configuration and lump them
into a single rigid extension attached to the distal end of link 3 (the
forearm). The combined "wrist cluster" contributes both extra mass and a
shift in the centre of mass of the effective forearm link.

All values come from Universal Robots' official kinematic / dynamic
datasheet for the UR5 (CB-series). Distances in metres, masses in kilograms,
inertias in kg*m^2 about the link's centre of mass, expressed in the link
frame.

Reference: Universal Robots, "Parameters for calculations of kinematics and
dynamics - UR5", available from universal-robots.com (Article 9355).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


# ---------------------------------------------------------------------------
# Modified Denavit-Hartenberg parameters for the first three joints.
# UR5 standard table; the negative link lengths come from the way UR5's
# kinematic frames are oriented.
# ---------------------------------------------------------------------------

# Joint axes / link lengths (metres)
D1 = 0.089159   # base height to shoulder
A2 = -0.425     # upper arm length (UR5 convention: signed)
A3 = -0.39225   # forearm length   (UR5 convention: signed)

# Wrist link DH offsets
D4 = 0.10915
D5 = 0.09465
D6 = 0.0823


# ---------------------------------------------------------------------------
# Individual link masses (kg) from the datasheet
# ---------------------------------------------------------------------------
M_BASE_SHOULDER = 3.7      # base + shoulder housing  (link 1)
M_UPPER_ARM     = 8.393    # upper arm                (link 2)
M_FOREARM       = 2.275    # forearm                  (link 3, bare)
M_WRIST1        = 1.219
M_WRIST2        = 1.219
M_WRIST3        = 0.1879


# ---------------------------------------------------------------------------
# Centres of mass in each link frame (metres, datasheet)
# ---------------------------------------------------------------------------
COM_LINK1 = np.array([0.0,        -0.02561,  0.00193])
COM_LINK2 = np.array([0.2125,      0.0,      0.11336])  # upper arm
COM_LINK3 = np.array([0.15,        0.0,      0.0265 ])  # forearm (bare)
COM_WRIST1 = np.array([0.0,       -0.0018,  0.01634])
COM_WRIST2 = np.array([0.0,        0.0018,  0.01634])
COM_WRIST3 = np.array([0.0,        0.0,    -0.001159])


# ---------------------------------------------------------------------------
# Inertia tensors about the COM, expressed in the link frame (kg*m^2).
# Datasheet provides diagonal entries (Ixx, Iyy, Izz).
# ---------------------------------------------------------------------------
def _diag(ixx: float, iyy: float, izz: float) -> np.ndarray:
    return np.diag([ixx, iyy, izz])

I_LINK1   = _diag(0.0084, 0.0064, 0.0084)
I_LINK2   = _diag(0.0078, 0.21,   0.21  )
I_LINK3   = _diag(0.0016, 0.0462, 0.0462)
I_WRIST1  = _diag(0.0016, 0.0016, 0.0009)
I_WRIST2  = _diag(0.0016, 0.0016, 0.0009)
I_WRIST3  = _diag(0.0001, 0.0001, 0.00005)


# ---------------------------------------------------------------------------
# DH transforms for the wrist chain (joints 4-6) at zero joint angles.
# T_{i-1, i} = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
# With a_i = 0 and theta_i = 0:
#   T_{i-1, i} = Tz(d_i) * Rx(alpha_i)
# ---------------------------------------------------------------------------
def _dh_zero(d: float, alpha: float) -> np.ndarray:
    """DH transform at a = 0, theta = 0."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [1.0, 0.0,  0.0,  0.0],
        [0.0, ca,  -sa,   0.0],
        [0.0, sa,   ca,   d  ],
        [0.0, 0.0,  0.0,  1.0],
    ])


T34_0 = _dh_zero(D4,  np.pi / 2)
T45_0 = _dh_zero(D5, -np.pi / 2)
T56_0 = _dh_zero(D6,  0.0)

T3_4 = T34_0
T3_5 = T3_4 @ T45_0
T3_6 = T3_5 @ T56_0


def _transform_point(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Apply a homogeneous transform to a 3-vector point."""
    return (T @ np.append(p, 1.0))[:3]


# ---------------------------------------------------------------------------
# Lump the wrist cluster (joints 4-6 frozen at zero) into link 3.
# ---------------------------------------------------------------------------
_com_w1_in_3 = _transform_point(T3_4, COM_WRIST1)
_com_w2_in_3 = _transform_point(T3_5, COM_WRIST2)
_com_w3_in_3 = _transform_point(T3_6, COM_WRIST3)

_masses = np.array([M_FOREARM, M_WRIST1, M_WRIST2, M_WRIST3])
_coms_in_3 = np.stack([COM_LINK3, _com_w1_in_3, _com_w2_in_3, _com_w3_in_3])
M3_EFF = float(_masses.sum())
COM_LINK3_EFF = (_masses[:, None] * _coms_in_3).sum(axis=0) / M3_EFF


def _lump_inertia() -> np.ndarray:
    """Combine forearm + wrist inertias about the lumped COM, in frame 3."""
    Itot = np.zeros((3, 3))
    # bare forearm (already in frame 3, no rotation)
    d = COM_LINK3 - COM_LINK3_EFF
    Itot += I_LINK3 + M_FOREARM * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    # wrist 1: rotate from frame 4 to frame 3
    R = T3_4[:3, :3]
    I_in_3 = R @ I_WRIST1 @ R.T
    d = _com_w1_in_3 - COM_LINK3_EFF
    Itot += I_in_3 + M_WRIST1 * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    # wrist 2
    R = T3_5[:3, :3]
    I_in_3 = R @ I_WRIST2 @ R.T
    d = _com_w2_in_3 - COM_LINK3_EFF
    Itot += I_in_3 + M_WRIST2 * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    # wrist 3
    R = T3_6[:3, :3]
    I_in_3 = R @ I_WRIST3 @ R.T
    d = _com_w3_in_3 - COM_LINK3_EFF
    Itot += I_in_3 + M_WRIST3 * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return Itot


I_LINK3_EFF = _lump_inertia()


# ---------------------------------------------------------------------------
# Final dataclass exposing the parameters used by dynamics / kinematics
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class UR5Params:
    """Parameters for the 3-DOF UR5 model (joints 1-3 active)."""

    d1: float = D1
    a2: float = A2
    a3: float = A3

    m1: float = M_BASE_SHOULDER
    m2: float = M_UPPER_ARM
    m3: float = M3_EFF              # forearm + wrist cluster, lumped

    com1: np.ndarray = field(default_factory=lambda: COM_LINK1.copy())
    com2: np.ndarray = field(default_factory=lambda: COM_LINK2.copy())
    com3: np.ndarray = field(default_factory=lambda: COM_LINK3_EFF.copy())

    I1: np.ndarray = field(default_factory=lambda: I_LINK1.copy())
    I2: np.ndarray = field(default_factory=lambda: I_LINK2.copy())
    I3: np.ndarray = field(default_factory=lambda: I_LINK3_EFF.copy())

    g: float = 9.81


def default_params() -> UR5Params:
    return UR5Params()


if __name__ == "__main__":
    p = default_params()
    print("UR5 3-DOF parameters")
    print("--------------------")
    print(f"d1 = {p.d1:.5f} m")
    print(f"a2 = {p.a2:.5f} m   (negative by UR5 convention)")
    print(f"a3 = {p.a3:.5f} m   (negative by UR5 convention)")
    print(f"m1 = {p.m1:.3f} kg  (base + shoulder)")
    print(f"m2 = {p.m2:.3f} kg  (upper arm)")
    print(f"m3 = {p.m3:.3f} kg  (forearm + lumped wrist)")
    print(f"COM1 = {p.com1}")
    print(f"COM2 = {p.com2}")
    print(f"COM3 (lumped) = {p.com3}")
    print(f"I3 (lumped) diag = {np.diag(p.I3)}")
