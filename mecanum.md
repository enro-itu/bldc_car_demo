# Pathway to a Mecanum Wheel Simulation

## 1) Pick your wheel model (realism vs. speed)

**Option A --- "Real" rollers (high realism, heavier CPU):** - Each
wheel = 8--12 passive rollers at **±45°**, each a small cylinder with
its own revolute joint. - Great traction behavior, but many contacts →
slower.

**Option B --- "Anisotropic friction" (fast, good enough for
controls):** - Each wheel = single cylinder; use **friction pyramid
anisotropy** so rolling direction has high friction and the lateral
(roller) direction has low friction. - Tune per physics engine (Gazebo
Harmonic supports directional friction; if engine lacks it, approximate
with slightly "rounded" wheels and low lateral μ).

> For most robotics control work, start with **Option B**. You can
> switch to A later if you want beautiful contact realism.

------------------------------------------------------------------------

## 2) Geometry & frames

-   Name your wheel joints consistently:
    -   `front_left_joint`, `front_right_joint`, `rear_left_joint`,
        `rear_right_joint`
-   Wheel radius: `r`
-   Half-wheelbase (front/back distance to CoG): `Lx`
-   Half-track (left/right distance to CoG): `Ly`
-   Use **body frame**: +x forward, +y left, +z up.

------------------------------------------------------------------------

## 3) Kinematics for mecanum (the core math)

With 45° rollers, the standard mapping (watch your signs; see notes
below):

\[
```{=tex}
\begin{bmatrix}
\omega_{FL} \\
\omega_{FR} \\
\omega_{RL} \\
\omega_{RR}
\end{bmatrix}
```
= `\frac{1}{r}`{=tex}
```{=tex}
\begin{bmatrix}
1 & -1 & -(L_x + L_y) \\
1 &  1 &  (L_x + L_y) \\
1 &  1 & -(L_x + L_y) \\
1 & -1 &  (L_x + L_y)
\end{bmatrix}
\begin{bmatrix}
v_x \\ v_y \\ \omega_z
\end{bmatrix}
```
\]

-   (v_x) forward, (v_y) left (strafe), (`\omega`{=tex}\_z) yaw CCW.
-   If your wheels spin "backwards" or strafe flips, **flip signs per
    wheel** in parameters (don't rewrite code).

------------------------------------------------------------------------

## 4) Topics & namespaces (recommended)

Use per-wheel namespaces to avoid collisions:

-   Commands (voltage):\
    `/mecanum/front_left/voltage`, `/mecanum/front_right/voltage`,
    `/mecanum/rear_left/voltage`, `/mecanum/rear_right/voltage`
-   States (Vector3: x=ω, y=I, z=τ):\
    `/mecanum/<wheel>/state`

Your improved BLDC plugin already supports this style (via
`<ros><namespace>…</namespace></ros>`, `<voltage_topic>`,
`<state_topic>`).

------------------------------------------------------------------------

## 5) SDF: attach the BLDC plugin to **all four** wheel joints

Example block (copy 4× and adjust `joint_name` + namespace):

``` xml
<plugin name="front_left_motor" filename="libbldc_motor_plugin.so">
  <joint_name>front_left_joint</joint_name>

  <!-- Electrical -->
  <resistance>0.5</resistance>
  <ke>0.02</ke>
  <kt>0.02</kt>
  <inductance>0.0002</inductance>

  <!-- I/O -->
  <use_ros_voltage_sub>true</use_ros_voltage_sub>
  <voltage_topic>voltage</voltage_topic>  <!-- relative -->
  <state_topic>state</state_topic>        <!-- relative -->
  <ros><namespace>/mecanum/front_left</namespace></ros>

  <!-- Limits & friction (tune) -->
  <voltage_limit_v>24.0</voltage_limit_v>
  <current_limit_a>10.0</current_limit_a>
  <torque_limit_nm>6.0</torque_limit_nm>
  <viscous_friction>0.01</viscous_friction>
  <coulomb_friction>0.05</coulomb_friction>
</plugin>
```

Repeat for `front_right_joint`, `rear_left_joint`, `rear_right_joint`
with namespaces `/mecanum/front_right`, `/mecanum/rear_left`,
`/mecanum/rear_right`.

------------------------------------------------------------------------

## 6) (Option B) Anisotropic friction tuning (fast model)

On each **wheel collision**:

-   Set a **high μ** along the rolling direction (tangent) and **low μ**
    along the lateral (roller) direction.
-   In Gazebo (Ignition) this is typically done by specifying two
    friction directions and coefficients (`mu`, `mu2`) aligned with the
    wheel frame. If your engine exposes direction vectors, set dir1
    along wheel tangential direction and dir2 lateral to it.
-   Start with: `mu ≈ 1.0–1.2` (rolling), `mu2 ≈ 0.05–0.2` (lateral).
    Tune until strafing is smooth and rotation is predictable.

> If your engine doesn't expose anisotropy, lower overall μ and rely on
> controller; or move to Option A rollers.

------------------------------------------------------------------------

## 7) Controller node: `/cmd_vel` → 4 wheel voltages

Take your current mapper and extend it to (v_x, v_y, `\omega`{=tex}\_z).
Keep your **feed-forward** voltage (V_i = K_v , `\omega`{=tex}\_i)
(start with (K_v `\approx `{=tex}K_e)). Add per-wheel sign flips in
params.

**Parameters** (ROS 2): - `wheel_radius` (r) - `Lx`, `Ly` - `kv`
(V·s/rad) - `vmax` (clamp) - Sign flips: `fl_sign`, `fr_sign`,
`rl_sign`, `rr_sign` (±1) - Topic names for each wheel's command

**Python sketch (drop-in mapper):**

``` python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MecanumMapper(Node):
    def __init__(self):
        super().__init__('mecanum_cmd_vel_to_voltage')

        # Params
        p = self.declare_parameter
        self.r  = p('wheel_radius', 0.08).value
        self.Lx = p('half_wheelbase', 0.20).value
        self.Ly = p('half_track',     0.18).value
        self.kv = p('kv',             0.02).value
        self.vmax = p('vmax',         24.0).value

        self.fl_sign = p('fl_sign', 1.0).value
        self.fr_sign = p('fr_sign', 1.0).value
        self.rl_sign = p('rl_sign', 1.0).value
        self.rr_sign = p('rr_sign', 1.0).value

        self.fl_topic = p('fl_topic', '/mecanum/front_left/voltage').value
        self.fr_topic = p('fr_topic', '/mecanum/front_right/voltage').value
        self.rl_topic = p('rl_topic', '/mecanum/rear_left/voltage').value
        self.rr_topic = p('rr_topic', '/mecanum/rear_right/voltage').value

        # Publishers
        self.pub_fl = self.create_publisher(Float64, self.fl_topic, 10)
        self.pub_fr = self.create_publisher(Float64, self.fr_topic, 10)
        self.pub_rl = self.create_publisher(Float64, self.rl_topic, 10)
        self.pub_rr = self.create_publisher(Float64, self.rr_topic, 10)

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.k_sum = self.Lx + self.Ly
        self.get_logger().info(
            f"Mecanum map /cmd_vel -> {self.fl_topic},{self.fr_topic},{self.rl_topic},{self.rr_topic} | "
            f"r={self.r}, Lx={self.Lx}, Ly={self.Ly}, kv={self.kv}, vmax={self.vmax}"
        )

    def clamp(self, v): return max(-self.vmax, min(self.vmax, v))

    def on_cmd(self, msg: Twist):
        vx = msg.linear.x      # forward (+)
        vy = msg.linear.y      # left (+)
        wz = msg.angular.z     # CCW (+)

        r, k = self.r, self.k_sum

        # wheel speeds (rad/s)
        w_fl = ( vx - vy - k*wz ) / r
        w_fr = ( vx + vy + k*wz ) / r
        w_rl = ( vx + vy - k*wz ) / r
        w_rr = ( vx - vy + k*wz ) / r

        # voltage feed-forward
        Vfl = self.clamp(self.fl_sign * self.kv * w_fl)
        Vfr = self.clamp(self.fr_sign * self.kv * w_fr)
        Vrl = self.clamp(self.rl_sign * self.kv * w_rl)
        Vrr = self.clamp(self.rr_sign * self.kv * w_rr)

        self.pub_fl.publish(Float64(data=Vfl))
        self.pub_fr.publish(Float64(data=Vfr))
        self.pub_rl.publish(Float64(data=Vrl))
        self.pub_rr.publish(Float64(data=Vrr))

def main():
    rclpy.init()
    node = MecanumMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

------------------------------------------------------------------------

## 8) Launch & params

Add a `mecanum.yaml`:

``` yaml
mecanum_cmd_vel_to_voltage:
  ros__parameters:
    wheel_radius: 0.08
    half_wheelbase: 0.20
    half_track: 0.18
    kv: 0.02
    vmax: 24.0
    fl_sign: 1.0
    fr_sign: 1.0
    rl_sign: 1.0
    rr_sign: 1.0
    fl_topic: "/mecanum/front_left/voltage"
    fr_topic: "/mecanum/front_right/voltage"
    rl_topic: "/mecanum/rear_left/voltage"
    rr_topic: "/mecanum/rear_right/voltage"
```

Update your launch to start: - Gazebo world with the mecanum car - The
mecanum mapper node (using the YAML above) - (Optional) A state bridge
if you want unified outputs (but not required)

------------------------------------------------------------------------

## 9) Test script (sanity)

After launching the sim:

``` bash
# Check topics
ros2 topic list | egrep 'mecanum|cmd_vel'

# States flowing?
ros2 topic echo /mecanum/front_left/state

# Pure forward (no strafe, no yaw)
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Pure strafe left
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {y: 0.5}, angular: {z: 0.0}}'

# Pure yaw CCW
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{angular: {z: 0.6}}'

# Diagonal + yaw
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.4, y: 0.2}, angular: {z: 0.4}}'
```

If motion directions are off, tweak `*_sign` or swap the `±vy` for the
misbehaving side.

------------------------------------------------------------------------

## 10) Tuning order (do this in order)

1)  **Mapper** only (no PI loops): get the car to move roughly right
    with feed-forward (K_v `\approx `{=tex}K_e).\
2)  **Friction**: tune anisotropy until strafing is smooth and rotation
    doesn't "stick".\
3)  **Limits**: set `<voltage_limit_v>` to bus voltage; pick safe
    `<current_limit_a>` and `<torque_limit_nm>`.\
4)  **Inductance** `L`: add a small value if current steps look
    unrealistically instant.\
5)  (Optional) Add **speed PI** at mapper level if you need high
    accuracy; otherwise voltage FF is fine.

------------------------------------------------------------------------

## 11) Common gotchas

-   **Wheels spin but no sideways motion** → lateral μ too high; reduce
    μ2 (or implement rollers).\
-   **Strafe OK but rotates weird** → bad (L_x, L_y) (sum too
    large/small) or a sign error on one wheel.\
-   **Jitter at low speed** → add small Coulomb + viscous friction;
    lower publish rate noise; ensure timestep stable.\
-   **Topic collisions** → always use per-wheel namespaces.\
-   **Left/right swapped** → swap the two front or two rear topics in
    params (or flip signs).
