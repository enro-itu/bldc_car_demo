# BLDC Car Demo (ROS 2 Jazzy + Gazebo)

A minimal differential-drive car simulation with BLDC motor physics.  
`/cmd_vel` is converted to voltage commands for each rear wheel, and the plugin computes current, torque, and angular velocity realistically.

---

**Prerequisites**

Ubuntu 24.04 (Noble)  
ROS 2 Jazzy  
Gazebo (gz-sim), colcon, git  
Keyboard teleop package

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop ros-jazzy-gazebo-ros-pkgs \
  python3-colcon-common-extensions git \
  ros-jazzy-teleop-twist-keyboard
```

---

**Workspace Setup**

```bash
# create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# car demo (this repo)
git clone -b Prototype https://github.com/enro-itu/bldc_car_demo.git

# BLDC Gazebo plugin
git clone -b Prototype_Car_Demo https://github.com/enro-itu/BLDCGazeboROS2.git
```

---

**Build**

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

---

**Source (do this in every terminal you use)**

```bash
# ROS 2
source /opt/ros/jazzy/setup.bash

# your workspace
source ~/ros2_ws/install/setup.bash
```

Make it automatic:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

---

**Gazebo Plugin Path**

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/install/bldc_gz_sim/lib
```

Persist:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/install/bldc_gz_sim/lib' >> ~/.bashrc
```

---

**Run (correct order)**

Open three terminals (A, B, C). Source in each.

**A) Launch the world + car**
```bash
ros2 launch bldc_car_demo sim_car.launch.py
```

**B) Start the cmd_vel → voltage mapper**
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
```
i/j/k/l/,  motion
q/z        change max speeds
w/x        linear only
e/c        angular only
```

---

**Topics & Data Flow**

```
/cmd_vel (Twist)
   ↓
cmd_vel_to_voltage
   ↓                 ↓
/rear_left/voltage_cmd    /rear_right/voltage_cmd
   ↓                 ↓
BLDCMotorPlugin L    BLDCMotorPlugin R
   ↓                 ↓
/rear_left/state          /rear_right/state
```

State message (`geometry_msgs/Vector3`):
- `x` = angular velocity ω [rad/s]
- `y` = current I [A]
- `z` = torque τ [N·m]

Example:
```
x: 7.157
y: 23.714
z: 0.474
```

---

**Monitor & Manual Tests**

View state:
```bash
ros2 topic echo /rear_left/state
ros2 topic echo /rear_right/state
```

Manual `/cmd_vel`:
```bash
# forward
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

# rotate left
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Manual voltages:
```bash
ros2 topic pub /rear_left/voltage_cmd  std_msgs/msg/Float64 "data: 12.0"
ros2 topic pub /rear_right/voltage_cmd std_msgs/msg/Float64 "data: 12.0"
```

---

**Motor Model**

```
I = (V - Ke·ω) / R
τ = Kt·I
```

As speed ω increases, back-EMF Ke·ω rises, current drops, torque decreases.  
Even at constant voltage, I, τ, and ω evolve to equilibrium.

---

**Mapper Parameters**

- `wheel_separation` (m)
- `wheel_radius` (m)
- `max_voltage` (V)
- `kv_vel` (V per rad/s)
- `ks_static` (V)
- `deadman_timeout` (s)
- `publish_rate_hz` (Hz)
- `left_cmd_topic`, `right_cmd_topic`

Example:
```bash
ros2 run bldc_car_demo cmd_vel_to_voltage --ros-args -p max_voltage:=18.0
```
