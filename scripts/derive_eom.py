"""
One-off symbolic derivation of the 3-DOF UR5 equations of motion.

Runs the sympy derivation (slow), prints the inertia, Coriolis and gravity
matrices in human-readable form for inclusion in the report.

Usage:
    python -m scripts.derive_eom
"""

from __future__ import annotations

import time

import numpy as np

from src.ur5_3dof.dynamics import build_dynamics, symbolic_dynamics
from src.ur5_3dof.parameters import default_params


def main() -> None:
    print("Running symbolic Lagrangian derivation ...")
    t0 = time.time()
    D, C, G = symbolic_dynamics()
    print(f"Done in {time.time() - t0:.1f} s.\n")

    print("D(q) (inertia matrix, sympy form):")
    print(D)
    print("\nC(q, q-dot) (Coriolis / centrifugal):")
    print(C)
    print("\nG(q) (gravity):")
    print(G)

    dyn = build_dynamics(default_params())
    q  = np.array([0.0, -np.pi / 4, np.pi / 3])
    qd = np.array([0.1, -0.05, 0.2])
    print("\nNumerical check at random pose / velocity:")
    print(f"  q  = {q}")
    print(f"  qd = {qd}")
    print(f"  M(q) =\n{dyn.M(q)}")
    print(f"  C(q,qd) qd = {dyn.C(q, qd) @ qd}")
    print(f"  G(q) = {dyn.G(q)}")


if __name__ == "__main__":
    main()
