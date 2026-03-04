# 개발 일지 — 2026-03-04

## 오늘의 목표

- ROS2 Humble + Gazebo Classic 기반 2륜 자가균형 로봇 시뮬레이션 완성
- Cascade PID 제어기 구현 및 Gazebo 연동 검증
- 시뮬레이션 구동 파이프라인 안정화

---

## 1. 시스템 구조 확정

### 최종 아키텍처

```
/imu/data  ──► theta(pitch) 추출
/odom      ──► current_vx 피드백
/cmd_vel   ──► 사용자 명령

[Velocity PID]  : (cmd_vx - current_vx) → theta_setpoint (±0.12 rad)
[Balance PID]   : (theta_setpoint - theta) → v_out (m/s)

v_out, cmd_wz ──► /cmd_vel_wheels ──► libgazebo_ros_diff_drive ──► 바퀴
```

### 토픽 구조

| Topic | Type | Role |
|-------|------|------|
| `/imu/data` | sensor_msgs/Imu | 피치각·각속도 수신 |
| `/odom` | nav_msgs/Odometry | 현재 선속도 피드백 |
| `/cmd_vel` | geometry_msgs/Twist | 사용자 속도 명령 |
| `/cmd_vel_wheels` | geometry_msgs/Twist | diff_drive 플러그인 입력 |

---

## 2. 주요 설계 결정

### (1) gazebo_ros2_control → libgazebo_ros_diff_drive 전환

**문제:** `gazebo_ros2_control` 플러그인이 URDF에 포함되었으나 `controller_manager` 서비스가 생성되지 않음. 장시간 디버깅에도 원인 불명.

**결정:** Gazebo Classic 내장 `libgazebo_ros_diff_drive` 플러그인으로 전환.

**이유:**
- diff_drive 플러그인은 Gazebo와 직접 통합되어 별도 controller_manager 불필요
- `/cmd_vel_wheels`(Twist) 입력, `/odom` 발행을 자동 처리
- 실제 로봇 포팅 시 ros2_control로 전환 예정 (config 파일 보존)

**XACRO 최종 플러그인 설정:**
```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <remapping>cmd_vel:=cmd_vel_wheels</remapping>
      <remapping>odom:=odom</remapping>
    </ros>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.14</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <publish_odom>true</publish_odom>
  </plugin>
</gazebo>
```

### (2) 컨트롤러 출력: effort → velocity

**변경 전:** effort(토크, Nm) 출력 → `/left_wheel_effort_controller/commands`
**변경 후:** linear velocity(m/s) 출력 → `/cmd_vel_wheels` (Twist)

diff_drive 플러그인이 속도(m/s) 명령을 받으므로 PID 출력 단위를 맞춤.

### (3) Gazebo pause/unpause 시퀀스

**문제:** 시뮬레이션 시작 후 컨트롤러가 준비되기 전(약 2~3초)에 로봇이 낙하.

**해결:** `sim.launch.py`에 순차 시작 로직 구현:
```
t=0s  : Gazebo 시작 (paused=true — 물리 동결)
t=2s  : balance_controller_node 시작
t=3s  : /unpause_physics 서비스 호출 → 물리 시작
```

컨트롤러가 IMU 수신 후 제어 루프가 안정된 뒤 물리 시뮬레이션이 시작되므로 초기 낙하 방지.

---

## 3. Cascade PID 제어기 구현

### PID 클래스 (include/balance_robot_controller/pid.hpp)

- Anti-windup: 출력 포화 시 적분항 누적 중단
- 게인 실시간 조회: `kp()`, `ki()`, `kd()` 메서드
- `reset()`: 적분항·이전 오차 초기화 (낙하 복구 시 호출)

### Cascade 구조 (balance_controller_node.cpp)

```
Outer loop (Velocity PID, 200 Hz):
  error  = cmd_vx - current_vx
  output = theta_setpoint  ∈ [-0.12, +0.12] rad

Inner loop (Balance PID, 200 Hz):
  error  = theta_setpoint - theta
  output = v_out           ∈ [-1.5, +1.5] m/s
```

### 안전장치

- `|theta| > 0.35 rad` (~20°): 모터 정지 + PID 리셋 + 낙하 경고 로그
- `dt > 0.5s` 또는 `dt <= 0`: 제어 루프 스킵 (타이머 이상 방지)
- IMU 수신 전 제어 루프 스킵 (`imu_received_` 플래그)

### PID 파라미터 (pid_params.yaml)

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `balance_kp` | 2.0 | 균형 루프 비례 |
| `balance_ki` | 0.05 | 균형 루프 적분 |
| `balance_kd` | 0.3 | 균형 루프 미분 |
| `vel_kp` | 0.5 | 속도 루프 비례 |
| `vel_ki` | 0.05 | 속도 루프 적분 |
| `vel_kd` | 0.01 | 속도 루프 미분 |
| `theta_max` | 0.35 rad | 낙하 판정 임계값 |
| `max_vel` | 1.5 m/s | 최대 바퀴 속도 |
| `vel_theta_max` | 0.12 rad | 속도 루프 최대 출력 |

---

## 4. 발생한 문제와 해결

| 문제 | 원인 | 해결 |
|------|------|------|
| `robot_description` YAML 파싱 에러 | launch에서 xacro 출력을 raw string으로 처리 | `ParameterValue(Command(['xacro', file]), value_type=str)` |
| `xacro $(find-pkg-share ...)` 오류 | xacro는 `$(find ...)` 문법 사용 | `$(find balance_robot_description)`으로 수정 |
| `gzserver exit code 255` | 이전 Gazebo 인스턴스가 백그라운드 실행 중 | `killall gzserver gzclient` |
| `GAZEBO_PLUGIN_PATH` 미설정 | gazebo_ros2_control 플러그인 못 찾음 | `~/.bashrc`에 `/opt/ros/humble/lib` 추가 |
| `controller_manager` 서비스 미생성 | gazebo_ros2_control 호환성 문제 | libgazebo_ros_diff_drive로 전환 |
| 스폰 직후 로봇 낙하 | 컨트롤러 준비 전 물리 시작 | Gazebo paused=true + unpause_physics 시퀀스 |
| GitHub 인증 실패 | 계정 비밀번호 대신 토큰 필요 | Personal Access Token(classic, repo 권한) 사용 |

---

## 5. 현재 상태

### 완료된 것

- [x] WIP 수학적 모델링 (Lagrangian, 선형화)
- [x] URDF/XACRO 모델 (본체, 바퀴, IMU, diff_drive 플러그인)
- [x] Gazebo 시뮬레이션 환경 (empty world, 로봇 스폰)
- [x] Cascade PID 제어기 C++ 구현
- [x] pause/unpause 시퀀스로 초기 낙하 방지
- [x] GitHub 저장소 연동 및 push

### 다음 단계 (Step 5)

- [ ] PID 게인 튜닝 (Ziegler-Nichols 또는 시행착오)
- [ ] 전방 이동 명령 테스트 (`/cmd_vel` linear.x)
- [ ] 회전 동작 테스트 (`/cmd_vel` angular.z)
- [ ] teleop_twist_keyboard 연동 조작 테스트

---

## 6. 실행 방법

```bash
cd ~/robot_project/ros2_ws
source install/setup.bash

# 시뮬레이션 전체 실행
ros2 launch balance_robot_bringup sim.launch.py

# 실시간 PID 조정
ros2 param set /balance_controller balance_kp 3.0

# 속도 명령
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# 토픽 상태 확인
ros2 topic echo /imu/data --once
ros2 topic echo /odom --once
```

---

## 7. 참고 자료

- [ROS2 Humble — gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [libgazebo_ros_diff_drive plugin API](http://gazebosim.org/tutorials?tut=ros_gzplugins#Differentialdrive)
- [Wheeled Inverted Pendulum 동역학](../docs/01_modeling.md)
