# mecanum_drive_cpp

4-wheel holonomic mecanum drive controller for the Studica VMX-Pi robot.
Full x/y/ω velocity control, lateral odometry, EKF sensor fusion, DS4
joystick teleop (with strafe), SLAM mapping, Nav2 free navigation, and
nav2_route route navigation.

C++ version — preferred for deployment. See `src/python/mecanum_drive/` for the
readable Python reference implementation.

## Architecture

```
DS4 (Bluetooth) ──→ joy_node ──→ /joy
                                    │
                    studica_control gamepad_component
                                    │  (LS-X → linear.y strafe, RS-X → angular.z)
                               /cmd_vel
                                    │
                    mecanum_drive_node ──→ /drive/m_N/cmd  (×4 wheels)
                    (holonomic inverse kinematics)

/drive/m_N/encoder (×4) ─┐
/imu ─────────────────────┴──→ odometry_node ──→ /odom (x, y, vx, vy, vyaw)
                                                      │
/odom ────────────────────────────────────────────────┤
/imu ─────────────────────────────────────────────────┴──→ ekf_node ──→ /odometry/filtered
                                                                     └──→ TF: odom → base_link

/scan (from lidar_merge_cpp) ──→ slam_toolbox   ──→ /map  (mapping only)
                             └──→ AMCL           ──→ TF: map → odom  (navigation)
                             └──→ costmaps       ──→ Nav2 planner + RPP controller
```

## Launch modes

### Manual drive (DS4 gamepad)

```bash
ros2 launch mecanum_drive_cpp manual_drive.launch.py
```

Pair the DS4 to the VMX via Bluetooth. Left stick Y: forward/backward.
Left stick X: strafe left/right. Right stick X: turn. Hold R1 for turbo.

### SLAM mapping

```bash
ros2 launch mecanum_drive_cpp mapping.launch.py
```

Drive around the full space, then save:

```bash
ros2 run nav2_map_server map_saver_cli -f $HOME/maps/my_map
```

SLAM Toolbox is pinned to CPU core 3 (`taskset -c 3`) so its Ceres solver
does not starve EKF and odometry on the Pi CM4.

### Nav2 free navigation

```bash
ros2 launch mecanum_drive_cpp navigation.launch.py map:=$HOME/maps/my_map.yaml
```

Nav2 sends only `linear.x` and `angular.z` — the mecanum drive node converts
these to holonomic wheel commands. Gamepad is disabled automatically.

Open RViz on your VM:
1. **"2D Pose Estimate"** — click+drag at the robot's actual position.
2. **"Nav2 Goal"** — click+drag on the map to send a goal.

### Route navigation (nav2_route)

```bash
ros2 launch mecanum_drive_cpp route_navigation.launch.py \
    map:=$HOME/maps/my_map.yaml \
    graph:=$HOME/maps/my_graph.geojson
```

Build the graph first with `graph_builder_cpp` — see its
[README](../graph_builder_cpp/README.md) for the full workflow.

## Configuration

| File | Purpose |
|---|---|
| `config/params.yaml` | Wheel wiring, geometry, odometry, EKF config |
| `config/studica_params.yaml` | Titan CAN config, gamepad axis mapping, motor scales |
| `config/slam_params.yaml` | SLAM Toolbox resolution, range, loop closure |
| `config/nav2_params.yaml` | Nav2 free navigation — footprint, costmaps, RPP, AMCL |
| `config/route_nav2_params.yaml` | Route navigation — same as nav2_params + route_server |
| `config/route_nav_bt.xml` | Behavior tree for route tracking (ComputeAndTrackRoute) |

### Motor wiring

Motor ports on this robot (`params.yaml`):

```yaml
front_left:  { titan: "drive", port: 1 }   # m_1 — no inversion
front_right: { titan: "drive", port: 3 }   # m_3 — inverted (see below)
rear_left:   { titan: "drive", port: 0 }   # m_0 — no inversion
rear_right:  { titan: "drive", port: 2 }   # m_2 — inverted (see below)
```

Right-side motors (`m_2`, `m_3`) are physically wired in reverse — inversion flags in `studica_params.yaml`:

```yaml
m_2:  { invert_motor: true, invert_encoder: true, invert_rpm: true }
m_3:  { invert_motor: true, invert_encoder: true, invert_rpm: true }
```

### Digital I/O

Buttons (pulled high — `false` when pressed):

| Name | VMX Pin | Topic |
|---|---|---|
| button_start | 8 | `/button_start/state` |
| button_reset | 9 | `/button_reset/state` |
| button_stop | 10 | `/button_stop/state` |

LEDs:

| Name | VMX Pin | Topic |
|---|---|---|
| led_start | 12 | `/led_start/cmd` |
| led_reset | 13 | `/led_reset/cmd` |
| led_stop | 14 | `/led_stop/cmd` |

Publish `true`/`false` (`std_msgs/Bool`) to an LED cmd topic to turn it on/off.

### Light tower

5-segment LED tower — one output active at a time (`studica_params.yaml`):

| VMX Pin | Segment |
|---|---|
| 0 | Continuous (white) |
| 1 | Red |
| 2 | Green |
| 3 | Yellow |
| 4 | Buzzer |

- Service `/light_tower/set` — strings: `"red"`, `"green"`, `"green:blink"`, `"yellow"`, `"yellow:blink_hw"`, `"buzzer:2.5"`, `"off"`
- Topic `/light_tower/state` (`std_msgs/String`) — current state

### Gamepad strafe axis (`studica_params.yaml`)

```yaml
gamepad:
  axis_linear_x:  1      # LS-Y — forward/backward
  axis_linear_y:  0      # LS-X — strafe
  axis_angular_z: 3      # RS-X — turn
  linear_scale:  -0.5    # negative: stick up = forward
  angular_scale:  1.0
```

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Input — linear.x/y + angular.z |
| `/odom` | `nav_msgs/Odometry` | Output — holonomic wheel odometry |
| `/odometry/filtered` | `nav_msgs/Odometry` | Output — EKF-fused odometry |

## Build

```bash
colcon build --packages-select mecanum_drive_cpp
source install/setup.bash
```

## Verify

```bash
# Watch position while driving forward
ros2 topic echo /odometry/filtered --field pose.pose.position

# Test strafe right — moves right for 1 s then stops
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {y: 0.2}, angular: {z: 0.0}}" --once && \
sleep 1 && \
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# TF tree
ros2 run tf2_tools view_frames
```
