# Balance Robot — ROS2 Humble Simulation

A self-balancing two-wheeled robot (Wheeled Inverted Pendulum) simulation built with **ROS2 Humble** and **Gazebo Classic**.

## Development Roadmap

| Step | Topic | Status |
|------|-------|--------|
| 1 | Physical & Mathematical Modeling | ✅ Done |
| 2 | URDF/XACRO Robot Modeling | ✅ Done |
| 3 | Gazebo Simulation & Plugins | ✅ Done |
| 4 | PID Controller Node (C++) | ✅ Done |
| 5 | Parameter Tuning & Drive Test | 🔄 In Progress |

---

## Package Structure

```
ros2_ws/src/
├── balance_robot_description/   # URDF/XACRO model + RViz config
│   ├── urdf/balance_robot.xacro
│   └── launch/display.launch.py
├── balance_robot_gazebo/        # Gazebo world, plugins, ros2_control config
│   ├── worlds/empty.world
│   ├── config/ros2_control.yaml
│   └── launch/gazebo.launch.py
├── balance_robot_controller/    # PID + velocity controller (C++)
│   ├── include/.../pid.hpp
│   └── src/balance_controller_node.cpp
└── balance_robot_bringup/       # Top-level launch & PID parameters
    ├── config/pid_params.yaml
    └── launch/sim.launch.py
```

---

## System Architecture

```
/imu/data  ──► theta (pitch angle)
/odom      ──► current_vx (velocity feedback)
/cmd_vel   ──► user command

[Velocity PID]  : (cmd_vx - current_vx) → theta_setpoint (±0.12 rad)
[Balance PID]   : (theta_setpoint - theta) → v_out (m/s)

v_out, cmd_wz ──► /cmd_vel_wheels ──► libgazebo_ros_diff_drive ──► wheels
```

### Control Strategy

| Loop | Input | Output | Rate |
|------|-------|--------|------|
| Balance (inner) | IMU pitch θ [rad] | Wheel linear velocity [m/s] | 200 Hz |
| Velocity (outer) | cmd_vel vs odom | Tilt setpoint [rad] | 200 Hz |

### Startup Sequence (pause → control → unpause)

```
t=0s  Gazebo starts (physics PAUSED)
t=2s  balance_controller_node starts
t=3s  /unpause_physics called → physics begins
```

---

## Requirements

```bash
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-xacro \
                 ros-humble-robot-state-publisher
```

---

## Quick Start

```bash
# 1. Build
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. Launch simulation (Gazebo + controller)
ros2 launch balance_robot_bringup sim.launch.py

# 3. (Optional) Launch with RViz
ros2 launch balance_robot_bringup sim.launch.py use_rviz:=true

# 4. Send drive command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# 5. Real-time PID tuning
ros2 param set /balance_controller balance_kp 90.0
```

---

## Mathematical Model

See [docs/01_modeling.md](docs/01_modeling.md) for:
- Wheeled Inverted Pendulum dynamics (Lagrangian mechanics)
- Inertia tensor calculations
- Linearized state-space equations
- PID and LQR control strategy

## Development Log

See [docs/02_dev_log.md](docs/02_dev_log.md) for:
- Plugin migration: gazebo_ros2_control → libgazebo_ros_diff_drive
- Cascade PID implementation details
- Startup sequence design (pause/unpause)
- Troubleshooting record
