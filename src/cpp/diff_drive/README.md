# diff_drive_cpp

4-motor differential drive controller for the Studica VMX-Pi robot.
Handles motor commands, wheel odometry, EKF sensor fusion, DS4 joystick
teleop, SLAM mapping, Nav2 free navigation, and nav2_route route navigation.

C++ version — preferred for deployment. See `src/python/diff_drive/` for the
readable Python reference implementation.

## Architecture

```
DS4 (Bluetooth) ──→ joy_node ──→ /joy
                                    │
                    studica_control gamepad_component
                                    │
                               /cmd_vel
                                    │
                        diff_drive_node ──→ /drive/m_N/cmd  (×4 motors, pwm mode)
                        diff_drive_node ──→ /drive/m_N/rpm_cmd  (×4 motors, velocity mode)

/drive/m_N/encoder (×4) ─┐
/imu ─────────────────────┴──→ odometry_node ──→ /odom
                                                      │
/odom ────────────────────────────────────────────────┤
/imu ─────────────────────────────────────────────────┴──→ ekf_node ──→ /odometry/filtered
                                                                     └──→ TF: odom → base_link

/scan (from lidar_merge_cpp) ──→ slam_toolbox   ──→ /map  (mapping only)
                             └──→ AMCL           ──→ TF: map → odom  (navigation)
                             └──→ costmaps       ──→ Nav2 planner + RPP controller
```

## Launch modes

Each launch file is self-contained — one command starts hardware, lidars, and
all relevant nodes.

### Manual drive (DS4 gamepad)

```bash
ros2 launch diff_drive_cpp manual_drive.launch.py
```

Pair the DS4 to the VMX via Bluetooth before launching.
Left stick: forward/backward. Right stick X: turn. Hold R1 for turbo.

### SLAM mapping

```bash
ros2 launch diff_drive_cpp mapping.launch.py
```

Drive around the entire space, then save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f $HOME/maps/my_map
# → writes my_map.pgm + my_map.yaml
```

SLAM Toolbox is pinned to CPU core 3 (`taskset -c 3`) so its Ceres solver
does not starve EKF and odometry on the Pi CM4.

### Nav2 free navigation

```bash
ros2 launch diff_drive_cpp navigation.launch.py map:=$HOME/maps/my_map.yaml
```

Starts the full Nav2 stack (AMCL + NavFn planner + RPP controller + costmaps).
Gamepad is disabled automatically — Nav2 owns `/cmd_vel`.

Open RViz on your VM and:
1. **"2D Pose Estimate"** — click+drag on the map at the robot's actual position.
2. **"Nav2 Goal"** — click+drag anywhere on the map to send a goal.

### Route navigation (nav2_route)

Follow a predefined waypoint graph instead of planning freely. Build the
graph first with `graph_builder_cpp`, then:

```bash
ros2 launch diff_drive_cpp route_navigation.launch.py \
    map:=$HOME/maps/my_map.yaml \
    graph:=$HOME/maps/my_graph.geojson
```

Obstacle avoidance and dynamic re-routing are handled by the
`nav2_route::CollisionMonitor` operation inside `route_server` — it checks
the global costmap every second and re-routes through alternate graph edges
when the planned path is blocked.

## Configuration

| File | Purpose |
|---|---|
| `config/params.yaml` | Motor wiring, odometry geometry, EKF config |
| `config/studica_params.yaml` | Titan CAN config, gamepad axis mapping, motor scales |
| `config/slam_params.yaml` | SLAM Toolbox resolution, range, loop closure |
| `config/nav2_params.yaml` | Nav2 free navigation — footprint, costmaps, RPP, AMCL |
| `config/route_nav2_params.yaml` | Route navigation — same as nav2_params + route_server |
| `config/route_nav_bt.xml` | Behavior tree for route tracking (ComputeAndTrackRoute) |

### Key params to tune (`nav2_params.yaml` / `route_nav2_params.yaml`)

| Parameter | Default | Notes |
|---|---|---|
| `footprint` | `445×390mm box` | Must match real robot. Update both global and local costmap. |
| `inflation_radius` | `0.08 m` | Clearance outside footprint. Max 0.105 m for 600mm court gaps. |
| `desired_linear_vel` | `0.12 m/s` | Cruise speed. Max hardware: 0.63 m/s. |
| `xy_goal_tolerance` | `0.10–0.25 m` | How close counts as goal reached. |

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

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Input — linear.x + angular.z |
| `/drive/m_N/cmd` | `std_msgs/Float64` | Output — duty cycle (when `drive_type: pwm`) |
| `/drive/m_N/rpm_cmd` | `std_msgs/Float64` | Output — target rpm (when `drive_type: velocity`) |
| `/odom` | `nav_msgs/Odometry` | Output — wheel odometry |
| `/odometry/filtered` | `nav_msgs/Odometry` | Output — EKF-fused odometry |

### Closed-loop velocity teleop (PID type 1 or 2)

1. In `studica_params.yaml`: `default_pid_type: 2` (or `1` for legacy)
2. In `params.yaml`: `drive_type: "velocity"` and tune `max_rpm`
3. Rebuild both `studica_control` and `diff_drive_cpp`, then launch as usual

Run MCV2 autotune before expecting good tracking (see below).

### MCV2 autotune

Requires PID type 2 on the motor(s) being tuned.

**USB (serial)**

| Command | When to use |
|---|---|
| `Autotune all ++--` | On the ground — symmetric back/forth; signs match robot motor layout |
| `Autotune all` | Lifted — one-direction forward sweep |
| `Autotune 0` | Single motor on the bench |

**ROS / CAN** (`titan_cmd` service on `studica_control`):

```bash
# Lifted robot — forward-only (same as Autotune all)
ros2 service call /titan0/titan_cmd studica_control/srv/SetData "{params: 'set_pid_type', initparams: {int_value: 2}}"
ros2 service call /titan0/titan_cmd studica_control/srv/SetData "{params: 'autotune', initparams: {}}"

# On the ground — symmetric sweep; uses signs from invert_motor in studica_params.yaml
ros2 service call /titan0/titan_cmd studica_control/srv/SetData "{params: 'autotune_symmetric', initparams: {}}"
```

Wait ~30–60 s after symmetric autotune (longer than lifted-only). 

On startup, `diff_drive` should log `drive_type=velocity` and topic lines ending in `/rpm_cmd`. If you still see `switch to pid type 0 before open-loop duty commands`, something is publishing to `/drive/m_N/cmd` (usually an old pwm-mode `diff_drive_node` still running):

```bash
ros2 param get /diff_drive drive_type
ros2 topic info /drive/m_0/cmd -v
```

## Build

```bash
colcon build --packages-select diff_drive_cpp
source install/setup.bash
```

## Verify

```bash
# Watch EKF output while driving forward
ros2 topic echo /odometry/filtered --field pose.pose.position

# Test a velocity command — drives forward for 1 s then stops
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once && \
sleep 1 && \
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Check TF tree
ros2 run tf2_tools view_frames
```
