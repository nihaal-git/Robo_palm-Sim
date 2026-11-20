# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`Ust_description` is a ROS 2 Humble `ament_python` package for a simulated 5-finger robotic hand ("UstRoboPalm"), run in Ignition Gazebo (Fortress, `ign_ros2_control`). The package lives at `~/new_ws/src/Ust_description`; the colcon workspace root is `~/new_ws`.

## Active files

The repo contains several older/alternate variants. The ones currently in use are:

- Launch: `launch/new_gz.launch.py`
- Robot description: `urdf/Ust.urdf`
- Controller config: `config/effort_trajectory_controller.yaml`
- World: `worlds/empty.sdf`

Legacy/alternate, not used by the active launch: `Ust.xacro` (+ `Ust.gazebo`, `Ust.trans`, `materials.xacro`, used by `gazebo.launch.py`), `ign_gz.launch.py`, `display.launch.py` (RViz only), `controllers.yaml`, `ust_controllers.yaml` (effort-controller experiment), `ros_gz_bridge_gazebo.yaml`.

## Commands

```bash
# Build (from workspace root). Rebuild after ANY change — launch, urdf, config and worlds are
# installed by setup.py data_files and read from install/share, not from src/.
cd ~/new_ws && colcon build --packages-select Ust_description && source install/setup.bash
# (--symlink-install avoids rebuilding for non-Python file edits)

# Run the sim (starts paused by default; pause_sim:=false to auto-run)
ros2 launch Ust_description new_gz.launch.py pause_sim:=false

# Run a node standalone
ros2 run Ust_description admittance_controller
ros2 run Ust_description test_data_sender

# Tests (ament lint only: flake8, pep257, copyright)
colcon test --packages-select Ust_description && colcon test-result --verbose
# single test
cd ~/new_ws/src/Ust_description && python3 -m pytest test/test_flake8.py
```

New nodes must be registered in `setup.py` `entry_points` to be runnable with `ros2 run`.

## Architecture

### Simulation / control wiring (spans URDF, YAML, launch)

1. `new_gz.launch.py` runs `xacro` over `Ust.urdf` (a plain URDF, but xacro is needed to resolve `$(find Ust_description)`), starts gz server (`-s`) and GUI client (`-g`) as separate `gz_sim.launch.py` includes, spawns the model via `ros_gz_sim create`, starts `robot_state_publisher`, spawns `joint_state_broadcaster`, `mcp_trajectory_controller` and `pip_dip_trajectory_controller`, and runs `joint_plotter`.
2. The URDF's `<ros2_control name="UstRoboPalmSystem">` block uses `ign_ros2_control/IgnitionSystem`. All 15 revolute joints have a **velocity** command interface plus position/velocity state. Unactuated joints swing freely when the MCP joints move, so every joint must be held by a controller.
3. The `<gazebo>` plugin `libign_ros2_control-system.so` at the bottom of the URDF points at `config/effort_trajectory_controller.yaml` — this is how controller_manager gets its config (no separate controller_manager node in the launch).
4. `effort_trajectory_controller.yaml` defines two `JointTrajectoryController`s with **velocity** command interface + PID gains (despite the "effort" filename): `mcp_trajectory_controller` (5 MCP joints) and `pip_dip_trajectory_controller` (10 PIP/DIP joints, holds them at their last goal, default spawn pose). Command interfaces in the YAML must match those declared in the URDF `<ros2_control>` block.
   - Keep PID `d: 0` in velocity mode: d acts on velocity error and with `d >= 1` the command flips sign every cycle (chatter). `i` has no effect unless `i_clamp` is set (defaults to 0).
   - The JTC checks goal tolerances on topic goals too: if any joint is outside `goal` tolerance (or moving faster than `stopped_velocity_tolerance`) at `goal_time` after the trajectory end, it logs `Exceeded goal_time_tolerance: holding position...` and freezes **all** its joints where they are. Unreachable targets (e.g. beyond a joint limit) trigger this.
   - A joint driven exactly onto its URDF **lower** limit locks there in Gazebo and does not respond to later goals. Keep targets slightly inside the limits (`admittance_controller` clamps with a 0.02 rad margin).
   - Streaming nodes must stay in step with the 100 Hz JTC cycle. A 50 Hz wall-clock ROS timer (the `ros2 run` default, no `use_sim_time`) drifts against it, so tracking chatters and joints overshoot onto the hard limit and lock (this was the "ring_mcp_joint not moving" bug). `adaptive_admittance_controller` therefore clocks its loop on `/joint_states` header stamps and brakes the reference near the soft limits (`limit_braking_gain`). Don't go back to a timer.

When changing control mode (e.g. velocity → effort/position), update the URDF `<ros2_control>` command interfaces, the controller YAML, and the spawner names in the launch file together.

Joint naming: `{pinky,ring,middle,index,thumb}_{mcp,pip,dip}_joint` (revolute) plus `*_bracket_fixed_joint` and `fixed_to_world`. Every revolute joint has a `force_torque` sensor; the world loads the ForceTorque system plugin. Gravity in `worlds/empty.sdf` is the normal `-9.81`.

URDF mesh paths are **absolute** `file:///home/user/new_ws/install/Ust_description/share/Ust_description/meshes/...` — the package must be built/installed for meshes to load, and the paths break if the workspace moves.

### Nodes (`Ust_description/`)

Assist-control data flow (current pipeline):

```
test_data_sender ──/emg (Float32)────────────────────────────┐
                 ├─/assist_torque_cmd (Float64MultiArray)────┤
                 └─/fsr (Float32MultiArray, not consumed yet)│
                                                             ▼
/joint_states ──────────────────────────────► admittance_controller ──► /mcp_trajectory_controller/joint_trajectory
                                                             └──► /admittance/debug_output
```

- `admittance_controller.py` — admittance model on the 5 MCP joints; turns external/EMG-scaled torque into desired positions sent as `JointTrajectory` to the trajectory controller.
- `test_data_sender.py` — synthetic EMG/FSR/assist-torque generator (parameters: rates, max torque, force soft/limit, noise).
- `joint_plotter.py` — subscribes `/joint_states`, plots position & velocity vs time for MCP joints, saves to `/tmp/joint_plot.png` (params: `topic`, `joints`, `window_seconds`, `save_path`).
- `synthetic_emg_publisher.py`, `synthetic_fsr_publisher.py`, `force_assist_controller.py` — earlier pipeline using different topics (`/emg/forearm_activation`, `/fsr/fingertip_force`, both `Float32`); not wired to the current admittance controller.

## Environment notes

- `venv/` in the package root is a local Python venv (gitignored); not required for the colcon build.
- `package.xml` does not yet declare all runtime deps used (e.g. `ros_gz_sim`, `controller_manager`, `ign_ros2_control`, `joint_trajectory_controller`, `trajectory_msgs`, `sensor_msgs`, matplotlib).
