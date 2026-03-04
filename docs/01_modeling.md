# Step 1: Physical & Mathematical Modeling

## Overview

The balance robot is a **Wheeled Inverted Pendulum (WIP)** — a classic control problem where a body balanced on two wheels must actively counteract gravity to remain upright.

---

## 1. System Parameters

These values are used as design targets. Inertia values are calculated analytically below.

| Symbol | Description | Value | Unit |
|--------|-------------|-------|------|
| `M_b` | Body (chassis) mass | 1.0 | kg |
| `M_w` | Wheel mass (each) | 0.1 | kg |
| `R` | Wheel radius | 0.05 | m |
| `W` | Wheel width | 0.02 | m |
| `L` | Distance from wheel axis to body CoM | 0.15 | m |
| `g` | Gravitational acceleration | 9.81 | m/s² |

---

## 2. Inertia Tensor Calculations

### Body (Rectangular Box: 0.1m × 0.08m × 0.28m)

Modeled as a solid rectangular box:

```
I_bx = (1/12) * M_b * (h² + d²)  = (1/12) * 1.0 * (0.28² + 0.08²) = 0.007227  [kg·m²]
I_by = (1/12) * M_b * (w² + d²)  = (1/12) * 1.0 * (0.10² + 0.08²) = 0.001367  [kg·m²]
I_bz = (1/12) * M_b * (w² + h²)  = (1/12) * 1.0 * (0.10² + 0.28²) = 0.007367  [kg·m²]
```

where w=0.10m (width), h=0.28m (height), d=0.08m (depth).

### Wheel (Solid Cylinder: R=0.05m, W=0.02m)

```
I_wroll = (1/2)  * M_w * R²       = (1/2)  * 0.1 * 0.05² = 0.000125  [kg·m²]  (spin axis)
I_wpitch = (1/4) * M_w * R²
         + (1/12)* M_w * W²       = 0.1*(0.0025/4 + 0.0004/12) = 0.0000658 [kg·m²]
I_wyaw  = I_wpitch                 = 0.0000658 [kg·m²]
```

---

## 3. Equations of Motion (State-Space)

The nonlinear dynamics of a WIP can be derived via Lagrangian mechanics.

**State vector:**

```
x = [θ, θ̇, x_pos, ẋ]
```

- `θ`     : body tilt angle (pitch) from vertical  [rad]
- `θ̇`    : tilt angular velocity               [rad/s]
- `x_pos` : horizontal displacement             [m]
- `ẋ`     : linear velocity                     [m/s]

**Linearized model (valid near θ ≈ 0):**

```
(M_b + 2*M_w) * ẍ + M_b*L * θ̈*cos(θ) = τ/R
M_b*L² * θ̈ - M_b*L*g*sin(θ) = -M_b*L * ẍ*cos(θ)
```

For small angles (sin θ ≈ θ, cos θ ≈ 1):

```
[M  m*L ] [ẍ ]   [1/R  ]       [0      ]
[m*L J  ] [θ̈] = [0    ] * τ + [m*L*g  ] * θ
```

Where:
- `M = M_b + 2*M_w` (total mass)
- `m = M_b`
- `J = I_bx + M_b*L²` (effective moment of inertia about wheel axis)

---

## 4. Control Target

| Condition | Target |
|-----------|--------|
| Upright balance | θ = 0 rad |
| Position hold | x_pos = const |
| Velocity command | ẋ from /cmd_vel |

The controller reads `θ` and `θ̇` from the **IMU** (`/imu/data` topic) and outputs **wheel torque** or **velocity** commands.

---

## 5. Control Strategy: PID (Phase 1) → LQR (Phase 2)

### PID (initial implementation)

```
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * ė(t)

e(t) = θ_target - θ_current  (tilt error)
```

Initial gain targets (to be tuned in simulation):
- `Kp` ≈ 50–100
- `Ki` ≈ 1–5
- `Kd` ≈ 5–15

### LQR (future improvement)

State feedback: `u = -K * x`

Cost matrices to be determined after system identification from simulation data.

---

## References

- Grasser, F. et al. (2002). *JOE: A Mobile, Inverted Pendulum*. IEEE Trans. on Industrial Electronics.
- ROS2 Control Documentation: https://control.ros.org
