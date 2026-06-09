# UR5: 3-DOF Dynamics and PID Control

Mini-project for **Robots Motion Planning and Control** (362-2-5481),
Spring 2026, Ben-Gurion University of the Negev. Lecturer: Prof. Amir
Shapiro.

**Students:** Itai Groisman (208394460), Daniel Zioni (ID TBD).

---

## What this project does

Simulates the first three joints (shoulder pan, shoulder lift, elbow) of a
Universal Robots UR5 manipulator. The remaining three wrist joints are
frozen at zero and lumped into a single rigid forearm extension, giving a
3-DOF spatial arm.

The pipeline:

1. **Symbolic dynamics.** M(q), C(q,q̇), G(q) derived with the Euler-Lagrange
   formalism using `sympy`. Analytical fast-path implementations
   (`mass_inertia_fast`, `coriolis_fast`, `gravity_fast`) are verified
   against the symbolic version to machine precision.
2. **Gazebo Harmonic simulation.** The robot runs with full physics through
   `gz_ros2_control`.
3. **Two controllers.** An independent-joint PID with gravity-compensation
   feedforward (`v1_aggressive`), and a computed-torque (feedback-
   linearisation) controller from Spong Ch. 8.
4. **Launch-time target selection.** The CTC launch accepts a `q_target`
   argument with built-in reachability validation.
5. **Automatic logging.** Each run saves a CSV and PNG plots to `results/`.

---

## Prerequisites

| Requirement | Version | Check command |
|---|---|---|
| Ubuntu | 24.04 | `lsb_release -a` |
| Python | 3.12 | `python3 --version` |
| ROS 2 | Jazzy | `ros2 --version` |
| Gazebo | Harmonic | `gz sim --version` |

Required ROS 2 packages:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ur-description \
  ros-jazzy-xacro \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz \
  ros-jazzy-robot-state-publisher
```

---

## Setup (run once after cloning)

### Step 1. Clone the repository

```bash
git clone https://github.com/itaigro/ur5-3dof-dynamics-pid.git
cd ur5-3dof-dynamics-pid
```

### Step 2. Install the Python library

```bash
pip install -e . --break-system-packages
```

This makes the `ur5_3dof` package importable from the ROS 2 nodes (they
load the analytical M, C, G fast paths from it).

### Step 3. Build the ROS 2 workspace

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
cd ..
```

Wait for `Summary: 1 package finished` with no errors.

### Step 4 (optional). Auto-source in every terminal

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ur5-3dof-dynamics-pid/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Without this you need both `source` lines in every new terminal.

---

## Running the controllers

Both launches start Gazebo paused, load controllers and the chosen
controller node, then unpause physics so the arm starts at the vertical
initial pose without falling.

### Controller A: PID v1 (independent-joint with gravity feedforward)

```bash
ros2 launch ur5_3dof_gz gazebo_pid_v1_aggressive.launch.py
```

Gains used:

| | q1 (base) | q2 (shoulder) | q3 (elbow) |
|---|---|---|---|
| Kp | 20 | 60 | 20 |
| Ki | 1 | 3 | 1 |
| Kd | 4 | 12 | 4 |

Expected behaviour: arm moves from vertical to the pickup target in
~3 s, with modest overshoot and peak torque around 75 N·m.

### Controller B: Computed-torque control (CTC)

```bash
ros2 launch ur5_3dof_gz gazebo_ctc.launch.py
```

The CTC implements

```
τ = M(q) · [q̈_d + Kd·(q̇_d − q̇) + Kp·(q_d − q)] + C(q,q̇)·q̇ + G(q)
```

with Kp = 16, Kd = 8 (critical damping at ω_n = 4 rad/s). Expected
behaviour: cleaner monotonic motion, settling in ~1.5 s, peak torque
around 17 N·m. This is the headline result of the project.

### Custom target pose

The CTC launch accepts a `q_target` argument with three joint values in
radians:

```bash
ros2 launch ur5_3dof_gz gazebo_ctc.launch.py \
  q_target:="[0.785, -1.047, -1.047]" \
  prefix:="ctc_wave"
```

Joint targets are validated at launch time. Targets that would cause
ground collisions or self-collisions are rejected with a clear error
message. The default target is the pickup pose `[π/2, -π/4, -π/2]`.

### Launch timeline (both controllers)

| Time | Event |
|---|---|
| 0 s | Gazebo opens **paused** |
| 3 s | Robot spawns at the vertical pose (still paused, so it stays put) |
| 10 s | Controllers (joint_state_broadcaster, arm_effort, wrist_position) load |
| 12 s | Torque bridge starts publishing zero effort |
| 13 s | Controller (PID or CTC) starts |
| 14 s | Gazebo unpaused. Motion begins. |

If you see the arm fall before t = 14 s, the pause did not engage (a
known race in `gz_ros2_control`). Kill everything (`pkill -9 -f "gz sim"`)
and relaunch.

### Stopping and saving plots

Press **Ctrl-C** in the launch terminal. The logger node saves files to
`results/`:

```
results/
├── <prefix>_trajectory.csv        : t, q1-3, qd1-3, tau1-3
├── <prefix>_joints_torques.png    : joint angles, velocities, torques vs time
└── <prefix>_stick_figure.png      : 3D snapshots of the arm pose
```

`<prefix>` defaults to `gazebo_pid_v1_aggressive` for PID and
`gazebo_ctc` for CTC, but can be overridden via the `prefix` launch
argument.

---

## Verify it is working

In a second terminal while a launch is running:

```bash
# All controllers should be active
ros2 control list_controllers

# Confirm torque commands are flowing
ros2 topic hz /torque_command

# Watch joint angles converge toward the target
ros2 topic echo /joint_states --field position
```

Expected `list_controllers` output:

```
wrist_position_controller  position_controllers/JointGroupPositionController  active
arm_effort_controller      effort_controllers/JointGroupEffortController       active
joint_state_broadcaster    joint_state_broadcaster/JointStateBroadcaster       active
```

---

## Report

The LaTeX source for the project report lives in `report/`. Compile with

```bash
cd report
latexmk -pdf main.tex
```

or upload the directory to Overleaf as a new project. See
`report/README.md` for details.

---

## Repository layout

```
ur5-3dof-dynamics-pid/
├── README.md                       # this file
├── PROJECT_JOURNAL.md              # design decisions and history
├── LICENSE
├── setup.py                        # installs the ur5_3dof Python package
├── src/ur5_3dof/                   # Core Python library
│   ├── parameters.py               # UR5 masses, DH parameters, inertias
│   ├── dynamics.py                 # Sympy + analytical fast paths
│   ├── kinematics.py               # Forward kinematics
│   └── visualization.py            # matplotlib plots and stick figure
├── scripts/
│   └── derive_eom.py               # One-off symbolic derivation
├── ros2_ws/src/
│   └── ur5_3dof_gz/                # The single ROS 2 package
│       ├── launch/
│       │   ├── gazebo_pid_v1_aggressive.launch.py
│       │   └── gazebo_ctc.launch.py
│       ├── ur5_3dof_gz/
│       │   ├── pid_node.py         # PID controller
│       │   ├── ctc_node.py         # Computed-torque controller
│       │   ├── torque_bridge_node.py
│       │   └── logger_node.py
│       ├── config/controllers.yaml
│       └── urdf/ur5_3dof_gz.urdf.xacro
├── report/                         # Overleaf-ready LaTeX
│   ├── main.tex
│   ├── references.bib
│   └── chapters/*.tex
├── results/                        # Output PNGs and CSVs (git-ignored)
└── docs/                           # Project spec and syllabus PDFs
```

---

## Troubleshooting

### "No controllers are currently loaded"

A previous launch is still running. Kill everything and relaunch:

```bash
pkill -9 -f "gz sim"; pkill -9 -f "gz_ros"; pkill -9 -f "robot_state"
pkill -9 -f "torque_bridge"; pkill -9 -f "ur5_pid"; pkill -9 -f "ur5_ctc"
pkill -9 -f "ur5_logger"
sleep 3
```

Then verify `ros2 node list` is empty before relaunching.

### "ModuleNotFoundError: No module named 'ur5_3dof'"

The Python library is not installed. From the repo root:

```bash
pip install -e . --break-system-packages
```

### Arm falls before t = 14 s

The Gazebo pause is not always reliable on first launch (known
`gz_ros2_control` issue). Kill everything and relaunch. If it keeps
happening, see `PROJECT_JOURNAL.md` for details and the workaround
options we considered.

### Custom q_target rejected at launch

The CTC launch validates that the target is reachable without ground
collision. The safe ranges are:

- q1: [-2π, 2π]
- q2: [-π, -0.1]  (must keep arm above horizontal)
- q3: [-2.5, 2.5]

Targets outside these typically cause the arm to drive into the ground.
Edit the `_SAFE_RANGES` table in `gazebo_ctc.launch.py` if you really
mean to go outside them.

### `colcon build` fails

Source ROS 2 before building:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ur5-3dof-dynamics-pid/ros2_ws
colcon build --symlink-install
```

---

## References

- M. Spong, S. Hutchinson, M. Vidyasagar, *Robot Modeling and Control*,
  Wiley, 2006.
- R. M. Murray, Z. Li, S. S. Sastry, *A Mathematical Introduction to
  Robotic Manipulation*, CRC Press, 1994.
- Universal Robots, *Parameters for calculations of kinematics and
  dynamics: UR5*, support article 9355.

## License

MIT. See `LICENSE`.
