# diff_drive (Python)

Python reference implementation of the differential drive stack.
Functionality is identical to [`diff_drive_cpp`](../../cpp/diff_drive/README.md)
— see that README for full configuration and tuning details.

The C++ package is preferred for deployment. This package exists so you can
read and understand the logic without wading through C++ boilerplate.

## Launch modes

```bash
# Manual joystick drive
ros2 launch diff_drive manual_drive.launch.py

# SLAM mapping
ros2 launch diff_drive mapping.launch.py

# Nav2 free navigation (requires a saved map)
ros2 launch diff_drive navigation.launch.py map:=$HOME/maps/my_map.yaml

# Route navigation (requires a saved map + GeoJSON graph)
ros2 launch diff_drive route_navigation.launch.py \
    map:=$HOME/maps/my_map.yaml \
    graph:=$HOME/maps/my_graph.geojson
```

## Architecture

```
DS4 → joy_node → /joy → studica_control gamepad → /cmd_vel
                                                       │
                                           diff_drive_node → motors

encoders + /imu → odometry_node → /odom → ekf_node → /odometry/filtered
                                                   └─→ TF: odom → base_link

/scan → slam_toolbox → /map + TF map→odom  (mapping.launch.py only)
/scan → AMCL + Nav2                        (navigation.launch.py / route_navigation.launch.py)
```

## Build

```bash
colcon build --packages-select diff_drive
source install/setup.bash
```

## Configuration

All config files mirror `diff_drive_cpp/config/` — see the
[C++ README](../../cpp/diff_drive/README.md) for parameter descriptions.

## Hardware wiring

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
