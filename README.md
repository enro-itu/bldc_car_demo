# BLDC Car (RWD) — Mapping & Workspace Sourcing

## 1) `/cmd_vel` → rear-wheel voltages (hinted mappings)

Let:
- `vx` = linear x (m/s)  
- `wz` = angular z (rad/s)  
- `R` = wheel radius (m)  
- `B` = half track (m)  *(half the distance between left/right wheels)*  
- `ωL, ωR` = target wheel angular speeds (rad/s)  
- `vmax` = bus voltage clamp (V)

### A. Kinematics → wheel speeds
```text
ωL = (vx - wz * B) / R
ωR = (vx + wz * B) / R
```

### B. Voltage mapping

Speed PI on each rear wheel (robust)
Subscribe to `/car/rear_*/state` (Vector3: x=ω, y=I, z=τ).  
Let `e = ω_target - ω_meas`. Add PI around A1:
```text
VL = clamp( Kv * ωL + Kp * eL + Ki * ∫eL dt, -vmax, +vmax )
VR = clamp( Kv * ωR + Kp * eR + Ki * ∫eR dt, -vmax, +vmax )
```
- Start small: `Kp ≈ 0.3..1.0`, `Ki ≈ 0.1*Kp` (tune in sim).  
- Keep an anti-windup: freeze integrator when |V|=vmax and sign would push further into saturation.

---

## 2) Minimal mapper node (skeleton)

```python
#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3  # for optional PI feedback

class RwdVoltageMapper(Node):
    def __init__(self):
        super().__init__('rwd_voltage_mapper')
        # Params
        self.declare_parameter('wheel_radius', 0.07)
        self.declare_parameter('half_track',  0.15)
        self.declare_parameter('Kv',          0.20)   # V per rad/s
        self.declare_parameter('vmax',        12.0)
        self.declare_parameter('use_pi',      True)
        self.declare_parameter('Kp',          0.6)
        self.declare_parameter('Ki',          0.1)

        p = self.get_parameter
        self.R   = float(p('wheel_radius').value)
        self.B   = float(p('half_track').value)
        self.Kv  = float(p('Kv').value)
        self.vmax= float(p('vmax').value)

        self.use_pi = bool(p('use_pi').value)
        self.Kp = float(p('Kp').value); self.Ki = float(p('Ki').value)

        # IO
        self.pub_L = self.create_publisher(Float64, '/car/rear_left/voltage', 10)
        self.pub_R = self.create_publisher(Float64, '/car/rear_right/voltage', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        # Optional feedback (ω from plugin state)
        self.omega_L = 0.0; self.omega_R = 0.0
        if self.use_pi:
            self.create_subscription(Vector3, '/car/rear_left/state',  self.on_state_L, 10)
            self.create_subscription(Vector3, '/car/rear_right/state', self.on_state_R, 10)
            self.iL = 0.0; self.iR = 0.0
            self.dt = 1.0/100.0
            self.create_timer(self.dt, self.control_tick)
            self.target = (0.0, 0.0)  # (ωL, ωR)

    def clamp(self, x): return max(-self.vmax, min(self.vmax, x))

    def on_state_L(self, msg): self.omega_L = msg.x
    def on_state_R(self, msg): self.omega_R = msg.x

    def on_cmd(self, msg: Twist):
        vx = msg.linear.x; wz = msg.angular.z
        wL = (vx - wz * self.B) / self.R
        wR = (vx + wz * self.B) / self.R
        if not self.use_pi:
            self.pub_L.publish(Float64(data=self.clamp(self.Kv*wL)))
            self.pub_R.publish(Float64(data=self.clamp(self.Kv*wR)))
        else:
            self.target = (wL, wR)  # PI runs in timer

    def control_tick(self):
        wL_t, wR_t = self.target
        eL = wL_t - self.omega_L
        eR = wR_t - self.omega_R
        self.iL += eL * self.dt
        self.iR += eR * self.dt
        VL = self.Kv*wL_t + self.Kp*eL + self.Ki*self.iL
        VR = self.Kv*wR_t + self.Kp*eR + self.Ki*self.iR
        # anti-windup (simple)
        satL = self.clamp(VL); satR = self.clamp(VR)
        if abs(VL) > self.vmax and ((VL>0 and eL>0) or (VL<0 and eL<0)): self.iL -= eL*self.dt
        if abs(VR) > self.vmax and ((VR>0 and eR>0) or (VR<0 and eR<0)): self.iR -= eR*self.dt
        self.pub_L.publish(Float64(data=satL))
        self.pub_R.publish(Float64(data=satR))

def main():
    rclpy.init(); rclpy.spin(RwdVoltageMapper()); rclpy.shutdown()
if __name__ == '__main__': main()
```

Use it as-is or strip the PI bits by setting `use_pi:=False`.

---

## 3) How to source the existing project inside the new one

### **Single workspace (simplest)**
Put both packages in the same workspace:
```
~/ws/
  src/bldc_motor_sim/         # given project (plugin, GUI)
  src/bldc_car_demo/          # your new package
```
Build & source:
```bash
cd ~/ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```
One build, one overlay; Gazebo finds `libbldc_motor_plugin.so` automatically.

---

## 4) Quick launch checklist

- In your car SDF, attach **two** plugin blocks only to **rear** joints:
  - `/car/rear_left/voltage`, `/car/rear_left/state`
  - `/car/rear_right/voltage`, `/car/rear_right/state`
- Start Gazebo + your mapper:
```bash
ros2 launch bldc_car_demo car_demo.launch.py
```
- Drive with teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- Verify:
```bash
ros2 topic hz /car/rear_left/state
ros2 topic echo /car/rear_left/state --rate 2
```

---

## 5) Notes (to avoid head-scratching)

- **Axes:** rear wheel joints must rotate about **Y** (`0 1 0`) so +ω → forward in +X.
- **Collisions:** use **cylinder** collisions for wheels (visual meshes are fine, but keep collisions primitive).
- **Stability:** keep `<damping>` on wheel joints and `b_viscous` in plugin SDF; clamp current/torque.
- **Model discovery:** if Gazebo says “plugin not found”, you didn’t source the plugin workspace.
- **Rates:** set plugin `publish_rate_hz ≈ 30–50`. PI timer at ~100 Hz is fine.

That’s it — they’ll have everything they need without getting lost.
