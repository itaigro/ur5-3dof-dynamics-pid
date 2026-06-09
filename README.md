# UR5 — 3-DOF Dynamics and PID Control

Mini-project for **Robots Motion Planning and Control** (362-2-5481), Spring 2026, Ben-Gurion University of the Negev. Lecturer: Prof. Amir Shapiro.

**Students:** Itai Groisman (208394460), Daniel Zioni (ID TBD).

---

## What this project does

Simulates the **first three joints of a Universal Robots UR5** manipulator (shoulder pan, shoulder lift, elbow). The remaining three wrist joints are frozen and lumped into a single rigid end-effector, giving a 3-DOF arm.

The full pipeline:

1. **Symbolic dynamics** — M(q), C(q,q̇), G(q) derived via the Euler–Lagrange formulation using `sympy`.
2. **Uncontrolled simulation** — arm falls from rest under gravity (τ = 0).
3. **PID controller** — independent-joint PID with gravity-compensation feedforward drives the arm to a target pose.
4. **ROS2 + Gazebo** — controller runs in Gazebo Harmonic with full physics. Plots saved automatically.
5. **ROS2 + RViz** — same controller but using a custom RK4 integrator instead of Gazebo, visualised live in RViz.

---

## Prerequisites

### System requirements

| Requirement | Version | Check command |
|---|---|---|
| Ubuntu | 24.04 | `lsb_release -a` |
| Python | 3.12 | `python3 --version` |
| ROS2 | Jazzy | `ros2 --version` |
| Gazebo | Harmonic | `gz sim --version` |

### Required ROS2 packages

If any of these are missing, install them:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ur-description \
  ros-jazzy-xacro \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2
```

---

## Setup (run once after cloning)

### Step 1 — Clone the repository

```bash
git clone https://github.com/itaigro/ur5-3dof-dynamics-pid.git
cd ur5-3dof-dynamics-pid
```

### Step 2 — Install the Python library

```bash
pip install -e . --break-system-packages
```

This makes the `ur5_3dof` package importable from both standalone scripts and ROS2 nodes.

### Step 3 — Build the ROS2 workspace

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
cd ..
```

Wait until you see `Summary: X packages finished [...]` with no errors. If you see errors, see [Troubleshooting](#troubleshooting).

### Step 4 — (Optional) Auto-source in every terminal

To avoid typing the `source` commands every time, add them to your shell profile:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ur5-3dof-dynamics-pid/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> If you skip this, you must run both `source` lines manually at the start of every new terminal before using ROS2 commands.

---

## Running the simulations

> **Before any ROS2 command**, make sure both lines are sourced in your terminal:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/ur5-3dof-dynamics-pid/ros2_ws/install/setup.bash
> ```

---

### Option A — Gazebo simulation (full physics)

The UR5 runs inside Gazebo Harmonic. The PID controller drives the arm to a target pose. Torque, velocity, and position plots are saved automatically when you stop the simulation.

#### Launch

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5-3dof-dynamics-pid/ros2_ws/install/setup.bash
ros2 launch ur5_3dof_gz gazebo_pid.launch.py
```

#### What to expect — startup timeline

| Time after launch | What happens | What you see |
|---|---|---|
| 0 s | Gazebo opens | Empty world |
| 3 s | Robot spawns | UR5 arm appears |
| 15 s | Controllers activate | Terminal prints `[spawner_*]: Successfully loaded controller` |
| 16 s | PID + logger start | Terminal prints `PID ready (pure PD until gravity model loads)` — arm is held immediately |
| ~36 s | Gravity model done | Terminal prints `Gravity model ready — feedforward enabled` — full PID kicks in |

> **First run only:** The sympy gravity model compiles in a background thread (~20 s). The PID holds the arm in place with pure PD control during this time, so the arm never falls uncontrolled. Every subsequent launch in the same session skips the compilation.

> **If the arm is not moving:** Check if Gazebo is paused. The ⏸ button at the bottom-left of the Gazebo window will be highlighted. Press **Space** to unpause.

#### Verify it is working (open a second terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5-3dof-dynamics-pid/ros2_ws/install/setup.bash

# Check all three controllers are active
ros2 control list_controllers

# Confirm torque commands are flowing (~100 Hz after t = 55 s)
ros2 topic hz /torque_command

# Watch joint angles converge toward [1.57, -1.77, -0.52] rad
ros2 topic echo /joint_states --field position
```

Expected `list_controllers` output:
```
wrist_position_controller  position_controllers/JointGroupPositionController  active
arm_effort_controller      effort_controllers/JointGroupEffortController       active
joint_state_broadcaster    joint_state_broadcaster/JointStateBroadcaster       active
```

#### Stop and save plots

Press **Ctrl-C** in the launch terminal. The logger node saves results to `results/`:

```
results/
├── gazebo_pid_trajectory.csv        — t, q1-3 (rad), qd1-3 (rad/s), tau1-3 (N·m)
├── gazebo_pid_joints_torques.png    — joint angles + torques vs time
└── gazebo_pid_stick_figure.png      — 3D stick-figure snapshots
```

---

### Option B — ROS2 + RViz simulation (no Gazebo)

Uses a built-in RK4 integrator instead of Gazebo. Faster startup (~20 s total). RViz opens automatically and shows the arm moving in real time.

#### Uncontrolled — arm falls under gravity for 1 s

```bash
ros2 launch ur5_3dof_sim uncontrolled.launch.py
```

#### PID-controlled — arm moves to target pose over 4 s

```bash
ros2 launch ur5_3dof_sim pid_control.launch.py
```

Press **Ctrl-C** when done. Plots and CSV are saved to `results/` automatically.

---

### Option C — Standalone Python scripts (no ROS2 needed)

Runs everything in plain Python without any ROS2 or Gazebo dependency.

```bash
cd ~/ur5-3dof-dynamics-pid
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

python -m scripts.run_uncontrolled    # gravity fall, τ = 0
python -m scripts.run_pid             # PID to target pose
```

Plots are saved to `results/`. The integrator used here is `scipy.solve_ivp` (adaptive RK45).

---

### Option D — Replay a saved trajectory in RViz

After running any simulation, replay the saved CSV file in RViz:

```bash
ros2 launch ur5_3dof_viz replay.launch.py trajectory:=$(realpath results/pid_trajectory.csv) loop:=true playback_rate:=0.5
```

Change `playback_rate` to speed up or slow down the replay.

---

## ROS2 topic graphs

### Gazebo launch (`ur5_3dof_gz`)

```
Gazebo (gz_ros2_control plugin)
  ├─ joint_state_broadcaster  ──/joint_states──▶  ur5_pid_node
  │                                               ur5_logger_node
  │                                               robot_state_publisher
  ├─ arm_effort_controller    ◀──/arm_effort_controller/commands──
  └─ wrist_position_controller◀──/wrist_position_controller/commands──

ur5_pid_node  ──/torque_command──▶  torque_bridge
torque_bridge ──▶  /arm_effort_controller/commands
              ──▶  /wrist_position_controller/commands
```

### RViz launch (`ur5_3dof_sim`)

```
ur5_sim_node  ──/joint_states──▶  ur5_pid_node
                                   ur5_logger_node
                                   robot_state_publisher ──▶ RViz
ur5_pid_node  ──/torque_command──▶ ur5_sim_node
```

---

## Troubleshooting

### "No controllers are currently loaded"

A previous launch is still running in the background (duplicate nodes). Kill all related processes and relaunch:

```bash
pkill -9 -f "gz sim"; pkill -9 -f "gz_ros"; pkill -9 -f "robot_state"; pkill -9 -f "torque_bridge"; pkill -9 -f "ur5_pid"
```

Wait 3 seconds, then check `ros2 node list` is empty before relaunching.

### "ModuleNotFoundError: No module named 'ur5_3dof'"

The Python library is not installed. Run from the project root:

```bash
pip install -e . --break-system-packages
```

### Arm appears but does not move

1. Check if Gazebo is paused — press **Space** to unpause.
2. Wait until Terminal 1 prints `Gravity model ready.` (~55 s after launch on first run).
3. Verify controllers are active: `ros2 control list_controllers`
4. Verify torques are flowing: `ros2 topic hz /torque_command`

### Arm oscillates and does not settle

Wait a few seconds — initial oscillation is normal as the arm falls under gravity before the PID kicks in. If it oscillates indefinitely, the PID gains may be too aggressive for the current simulation step size. Try lowering `update_rate` in `ros2_ws/src/ur5_3dof_gz/config/controllers.yaml` from 100 to 50 Hz, then rebuild.

### Plots are not saved

The logger saves on **Ctrl-C**. If you closed the terminal window instead of pressing Ctrl-C, the save is skipped. Rerun the simulation and use Ctrl-C to stop.

### `colcon build` fails

Make sure ROS2 is sourced before building:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ur5-3dof-dynamics-pid/ros2_ws
colcon build --symlink-install
```

---

## Repository layout

```
ur5-3dof-dynamics-pid/
├── src/ur5_3dof/                  # Core Python library
│   ├── parameters.py              # UR5 link masses, DH parameters, inertias
│   ├── dynamics.py                # Lagrangian → M(q), C(q,qd), G(q) via sympy
│   ├── kinematics.py              # Forward kinematics for stick-figure plots
│   ├── simulator.py               # scipy solve_ivp integration wrapper
│   ├── controllers.py             # PID + gravity feedforward + anti-windup
│   └── visualization.py           # matplotlib plots and 3D stick-figure
├── scripts/
│   ├── run_uncontrolled.py        # Standalone: gravity fall (τ = 0)
│   ├── run_pid.py                 # Standalone: PID to target pose
│   └── make_video.py             # Render result animations to MP4
├── ros2_ws/src/
│   ├── ur5_3dof_gz/               # Gazebo Harmonic simulation
│   │   ├── launch/gazebo_pid.launch.py
│   │   ├── config/controllers.yaml
│   │   ├── urdf/ur5_3dof_gz.urdf.xacro
│   │   └── ur5_3dof_gz/torque_bridge_node.py
│   ├── ur5_3dof_sim/              # ROS2 + RViz simulation
│   │   ├── launch/pid_control.launch.py
│   │   ├── launch/uncontrolled.launch.py
│   │   ├── ur5_3dof_sim/sim_node.py
│   │   ├── ur5_3dof_sim/pid_node.py
│   │   └── ur5_3dof_sim/logger_node.py
│   └── ur5_3dof_viz/              # CSV replay in RViz
│       └── launch/replay.launch.py
├── results/                       # Output PNGs and CSVs (git-ignored)
├── tests/                         # Energy conservation and FK unit tests
├── docs/                          # Project spec and course syllabus PDFs
└── report/                        # LaTeX report (Overleaf-ready)
```

---

## References

- M. Spong, S. Hutchinson, M. Vidyasagar, *Robot Modeling and Control*, Wiley, 2006.
- R. M. Murray, Z. Li, S. S. Sastry, *A Mathematical Introduction to Robotic Manipulation*, CRC Press, 1994.
- Universal Robots, *Parameters for calculations of kinematics and dynamics — UR5*, UR support article 9355.

## License

MIT — see `LICENSE`.
