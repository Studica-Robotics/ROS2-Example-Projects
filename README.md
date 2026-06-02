# Studica Robotics — User Example Workspace

Example ROS2 packages for building robot applications on top of
[`studica_control`](https://github.com/Studica-Robotics/ROS2).
Python and C++ versions of every drive package are provided side-by-side.

## Hardware

- **Platform**: Studica VMX
- **OS**: Studica VMX OS image (ROS2 Humble pre-installed)
- **Motors**: Studica Titan motor controllers (CAN ID 10)
- **Lidars**: 2× YDLidar Tmini Plus (front `/dev/ttyUSB0`, back `/dev/ttyUSB1`)

## Workspace layout

```
src/
  cpp/                       # C++ packages — preferred for deployment
    diff_drive_cpp/          # differential drive + Nav2 + route navigation
    mecanum_drive_cpp/       # mecanum holonomic drive + Nav2 + route navigation
    lidar_merge_cpp/         # dual-lidar merge + confidence scan gate
    graph_builder_cpp/       # interactive route graph builder for nav2_route

  python/                    # Python packages — readable reference implementations
    diff_drive/              # mirrors diff_drive_cpp
    mecanum_drive/           # mirrors mecanum_drive_cpp
```

## Packages

| Package | Description |
|---|---|
| [`diff_drive_cpp`](src/cpp/diff_drive/) | 4-motor differential drive, odometry, EKF, DS4 teleop, SLAM mapping, Nav2 free navigation, nav2_route route navigation |
| [`mecanum_drive_cpp`](src/cpp/mecanum_drive/) | 4-wheel holonomic mecanum drive, full x/y/ω odometry, EKF, DS4 teleop (LS-X strafe), SLAM mapping, Nav2 free navigation, nav2_route route navigation |
| [`lidar_merge_cpp`](src/cpp/lidar_merge/) | Merges front and back YDLidar Tmini Plus scans into one `/scan`; confidence scan gate for SLAM quality control |
| [`graph_builder_cpp`](src/cpp/graph_builder_cpp/) | Interactive RViz-based route graph builder — click waypoints, connect edges, save GeoJSON for nav2_route |

## Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-joy \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-route \
  nlohmann-json3-dev
```

`ydlidar_ros2_driver` and `studica_control` must be installed first:
- https://github.com/Studica-Robotics/ROS2
- https://github.com/YDLIDAR/ydlidar_ros2_driver (pre-installed on VMX OS image)

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Partial builds:

```bash
colcon build --packages-select diff_drive_cpp mecanum_drive_cpp lidar_merge_cpp graph_builder_cpp
colcon build --packages-select diff_drive mecanum_drive   # Python
```

## Running

### Differential drive

| Goal | C++ | Python |
|---|---|---|
| Manual drive (DS4) | `ros2 launch diff_drive_cpp manual_drive.launch.py` | `ros2 launch diff_drive manual_drive.launch.py` |
| Build a SLAM map | `ros2 launch diff_drive_cpp mapping.launch.py` | `ros2 launch diff_drive mapping.launch.py` |
| Nav2 free navigation | `ros2 launch diff_drive_cpp navigation.launch.py map:=$HOME/maps/my_map.yaml` | `ros2 launch diff_drive navigation.launch.py map:=...` |
| Route navigation | `ros2 launch diff_drive_cpp route_navigation.launch.py map:=... graph:=...` | `ros2 launch diff_drive route_navigation.launch.py map:=... graph:=...` |

### Mecanum drive

| Goal | C++ | Python |
|---|---|---|
| Manual drive (DS4) | `ros2 launch mecanum_drive_cpp manual_drive.launch.py` | `ros2 launch mecanum_drive manual_drive.launch.py` |
| Build a SLAM map | `ros2 launch mecanum_drive_cpp mapping.launch.py` | `ros2 launch mecanum_drive mapping.launch.py` |
| Nav2 free navigation | `ros2 launch mecanum_drive_cpp navigation.launch.py map:=$HOME/maps/my_map.yaml` | `ros2 launch mecanum_drive navigation.launch.py map:=...` |
| Route navigation | `ros2 launch mecanum_drive_cpp route_navigation.launch.py map:=... graph:=...` | `ros2 launch mecanum_drive route_navigation.launch.py map:=... graph:=...` |

### Route graph builder

Build a graph once after mapping — only needs the saved map, not a running robot:

```bash
ros2 launch diff_drive_cpp navigation.launch.py map:=$HOME/maps/my_map.yaml
ros2 run graph_builder_cpp graph_builder \
    --ros-args -p save_path:=$HOME/maps/my_graph.geojson
```

See [graph_builder_cpp README](src/cpp/graph_builder_cpp/README.md) for the full workflow.

## Deploying to the robot

Copy changed source files to `~/ros2_ws/src/` on the robot (WinSCP or scp), then rebuild:

```bash
cd ~/ros2_ws
colcon build --packages-select diff_drive_cpp
source install/setup.bash
```