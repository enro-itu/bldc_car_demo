# BLDC Car Demo (ROS 2 Jazzy + Gazebo)

A minimal differential-drive car simulation with BLDC motor physics. `/cmd_vel` is converted to voltage commands for each rear wheel, and the plugin computes current, torque, and angular velocity realistically.

---

## 1. Prerequisites

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Gazebo (gz-sim)
- colcon, git
- Keyboard teleop package

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop ros-jazzy-gazebo-ros-pkgs \
  python3-colcon-common-extensions git \
  ros-jazzy-teleop-twist-keyboard
```

---

## 2. Workspace Setup

1. Create workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the car demo package:
```bash
git clone -b Prototype https://github.com/enro-itu/bldc_car_demo.git
```

3. Clone the BLDC motor plugin package (required):
```bash
git clone -b Prototype_Car_Demo https://github.com/enro-itu/BLDCGazeboROS2.git
```

> **Note:** The BLDC motor plugin is required for this demo to work. See [BLDCGazeboROS2](https://github.com/enro-itu/BLDCGazeboROS2) and its [plugin source](https://github.com/enro-itu/BLDCGazeboROS2/blob/main/src/bldc_motor_plugin.cpp) for details.

---

## 3. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

If the build fails, fix errors and re-run `colcon build`.

---

## 4. Source (every terminal)

This is the most common pitfall. If you forget this, `ros2 run`/`ros2 launch` won’t find your packages.

```bash
# ROS 2
source /opt/ros/jazzy/setup.bash

# your workspace
source ~/ros2_ws/install/setup.bash
```

To make this automatic in new terminals:
```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

---

## 5. Gazebo Plugin Path

Ensure Gazebo can find the BLDC system plugin:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/install/bldc_gz_sim/lib
```

(Optional) Persist it:
```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/install/bldc_gz_sim/lib' >> ~/.bashrc
```

---

## 6. Run (correct order)

Open three terminals (A, B, C). In each one, run the Source commands above.

**A) Launch the world + car**
```bash
ros2 launch bldc_car_demo sim_car.launch.py
```

**B) Start the cmd_vel → voltage mapper (must be running)**
```bash
ros2 run bldc_car_demo cmd_vel_to_voltage
```

Expected output:
```
[INFO] [cmd_vel_to_voltage]: Mapping /cmd_vel -> /rear_left/voltage_cmd, /rear_right/voltage_cmd
```

**C) Start keyboard teleop**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys:
- i / ,  : forward / backward
- j / l  : turn left / right
- q / z  : increase / decrease max speeds
- w / x  : adjust linear speed
- e / c  : adjust angular speed

---

## 7. Topics & Data Flow

```
/cmd_vel (Twist)
    ↓
cmd_vel_to_voltage
    ↓                 ↓
/rear_left/voltage_cmd   /rear_right/voltage_cmd
    ↓                         ↓
BLDCMotorPlugin (left)   BLDCMotorPlugin (right)
    ↓                         ↓
/rear_left/state            /rear_right/state
```

- `/cmd_vel` → `geometry_msgs/Twist`
- `/rear_left/voltage_cmd` → `std_msgs/Float64`
- `/rear_right/voltage_cmd` → `std_msgs/Float64`
- `/rear_left/state` & `/rear_right/state` → `geometry_msgs/Vector3`

`state` message:
- **x** = angular velocity ω [rad/s]
- **y** = current I [A]
- **z** = torque τ [N·m]

Values are formatted to 3 decimals.

---

## 8. Monitor & Manual Tests

View state:
```bash
ros2 topic echo /rear_left/state
ros2 topic echo /rear_right/state
```

Example:
```
x: 7.157
y: 23.714
z: 0.474
```

Manual `/cmd_vel` (without teleop):
```bash
# forward
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# rotate left
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Manual voltage commands (bypass mapper):
```bash
ros2 topic pub /rear_left/voltage_cmd  std_msgs/msg/Float64 "data: 12.0"
ros2 topic pub /rear_right/voltage_cmd std_msgs/msg/Float64 "data: 12.0"
```

---

## 9. Motor Model (per wheel)

```
I = (V - Ke * ω) / R
τ = Kt * I
```

As speed rises, `Ke·ω` increases → effective current drops → torque decreases.

Even at constant voltage `V`, current `I`, torque `τ`, and angular velocity `ω` evolve until they reach an electrical–mechanical equilibrium.

---

## 10. Parameters (mapper: cmd_vel_to_voltage)

You can override via YAML or CLI params:
- `wheel_separation` (m)
- `wheel_radius` (m)
- `max_voltage` (V)
- `kv_vel` (V per rad/s, mapping gain)
- `ks_static` (V, static offset)
- `deadman_timeout` (s)
- `publish_rate_hz` (Hz)
- `left_cmd_topic`, `right_cmd_topic` (strings)

Example run with custom param:
```bash
ros2 run bldc_car_demo cmd_vel_to_voltage --ros-args -p max_voltage:=18.0
```
