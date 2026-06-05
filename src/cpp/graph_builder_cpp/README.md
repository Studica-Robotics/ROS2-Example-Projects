# graph_builder_cpp

Interactive route graph builder for [nav2_route](https://github.com/ros-navigation/navigation2/tree/main/nav2_route).

Click waypoints in RViz on top of a live or saved map, connect them into a
graph, and save as GeoJSON. The saved file is passed directly to `route_server`
as the navigation graph.

## Workflow

### 1. Build a map first

```bash
ros2 launch diff_drive_cpp mapping.launch.py
# drive around the whole space, then save:
ros2 run nav2_map_server map_saver_cli -f $HOME/maps/my_map
```

### 2. Launch the graph builder

Start navigation with the saved map (so the map is visible in RViz), then
run the graph builder alongside:

```bash
# Terminal 1
ros2 launch diff_drive_cpp navigation.launch.py map:=$HOME/maps/my_map.yaml

# Terminal 2
ros2 run graph_builder_cpp graph_builder \
    --ros-args -p save_path:=$HOME/maps/my_graph.geojson
```

### 3. Set up RViz

1. Open RViz on your VM.
2. **Add → By topic → `/map` → Map.** In the Map display, set **QoS → Durability → Transient Local** so the map is visible (the map server latches the topic; Best Effort / Volatile will show nothing).

   ![Adding map in RViz](doc/adding%20map%20to%20rviz2.png)
   ![Setting Durability to Transient Local](doc/setting%20durablity%20policy%20to%20transient%20local.png)

3. **Add → By display type  → MarkerArray and set topic to → /graph_builder/MarkerArray/**

   ![Adding MarkerArray](doc/Adding%20marker%20array.png)
   ![Set topic to graph_builder](doc/set%20topic%20to%20graph_builder.png)

4. Select the **Publish Point** tool in the RViz toolbar (crosshair icon, shortcut **`g`**).

   ![Select Publish Point tool](doc/select%20publish%20point.png)

### 4. Place nodes

Click anywhere on the map. Each click places a waypoint node. With
`auto_connect: true` (default), each new node is automatically connected to
the previous one with a bidirectional edge (green line) — useful for tracing a path sequentially.

`auto_connect` only chains consecutive nodes. To connect all node pairs that
have clear line-of-sight, run the `autoconnect` command separately after placing
all nodes (see step 5).

![Adding points to map](doc/adding%20points%20to%20map.png)
![Points showing in terminal](doc/points%20showing%20in%20terminal.png)

### 5. Edit the graph

Most common commands — copy and run in a terminal:

```bash
# Run line-of-sight autoconnect across all placed nodes (cyan edges)
ros2 topic pub /graph_builder/cmd std_msgs/msg/String 'data: "autoconnect"' --once

# Manually add a bidirectional edge between two nodes
ros2 topic pub /graph_builder/cmd std_msgs/msg/String 'data: "connect 2 5"' --once

# Remove an edge
ros2 topic pub /graph_builder/cmd std_msgs/msg/String 'data: "disconnect 2 5"' --once

# Undo the last action
ros2 topic pub /graph_builder/cmd std_msgs/msg/String 'data: "undo"' --once

# Save the graph
ros2 topic pub /graph_builder/cmd std_msgs/msg/String 'data: "save"' --once
```

Full command reference:

| Command | Description |
|---|---|
| `save [path]` | Save graph to GeoJSON (default: `save_path` param) |
| `load <path>` | Load and display an existing GeoJSON |
| `connect <a> <b>` | Add a bidirectional edge between nodes a and b |
| `connect_one <a> <b>` | Add a directed edge a → b only |
| `disconnect <a> <b>` | Remove edge between a and b |
| `delete <id>` | Remove node and all its attached edges |
| `undo` | Remove the last added node or edge |
| `clear` | Remove everything and reset |
| `autoconnect` | Add edges between all node pairs with clear line-of-sight |
| `auto on\|off` | Toggle auto-connect on new node click (default: on) |
| `name <id> <text>` | Set a display name for node id |
| `info` | Print node and edge summary to terminal |

### 6. Auto-connect (line-of-sight)

`autoconnect` checks every pair of nodes for a clear path on the occupancy
map using Bresenham ray tracing. A robot-width clearance (195 mm each side) is
required along the entire line. Edges that pass through walls or unknown space
are not added.

Auto-connected edges appear **cyan** in RViz. Manual edges appear **green**
(bidirectional) or **yellow** (one-way).

![Auto-connect run with edges added](doc/auto%20connect%20is%20run%20and%20more%20edges%20added.png)

### 7. Save and use

```bash
ros2 topic pub /graph_builder/cmd std_msgs/msg/String 'data: "save"' --once
```

![Save command confirmed in terminal](doc/saved%20command%20run%20and%20confirmed%20in%20terminal.png)

Then launch route navigation with the saved graph:

```bash
ros2 launch diff_drive_cpp route_navigation.launch.py \
    map:=$HOME/maps/my_map.yaml \
    graph:=$HOME/maps/my_graph.geojson
```

## RViz marker colours

| Colour | Meaning |
|---|---|
| Blue sphere | Waypoint node |
| Orange sphere | Most recently placed node (next auto-connect target) |
| Green line | Bidirectional edge (manually added) |
| Yellow line + chevron | One-way directed edge |
| Cyan line | Bidirectional edge added by `autoconnect` |
| White label | Node ID and name |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `save_path` | `~/maps/graph.geojson` | Output GeoJSON file path |
| `auto_connect` | `true` | Chain-connect new nodes to the previous one |
| `map_frame` | `map` | Coordinate frame for RViz markers |

Set at launch:
```bash
ros2 run graph_builder_cpp graph_builder \
    --ros-args -p save_path:=/home/vmx/maps/court.geojson \
               -p auto_connect:=false
```

Or at runtime:
```bash
ros2 param set /graph_builder auto_connect false
```

## GeoJSON format

The saved file uses the nav2_route GeoJSON schema:

- **Point features** — waypoint nodes with `id`, `name`, `frame` properties.
- **MultiLineString features** — directed edges with `id`, `startid`, `endid`.
  Bidirectional edges emit two directed features (forward and reverse).

The graph builder can also load GeoJSON files saved by a previous session
(`load <path>`) and continue editing them.

## Build

```bash
colcon build --packages-select graph_builder_cpp
source install/setup.bash
```
