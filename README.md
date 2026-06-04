# 2D-Scan Merger (ROS2)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%20|%20Iron%20|%20Jazzy%20|%20Kilted-blue)](https://ros.org)
[![C++](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://isocpp.org)

A ROS2 composable node that merges any number of 2D LiDAR scans into a single unified
`sensor_msgs/LaserScan`, with approximate-time synchronization and parallel ray projection.

![2D-Scan Merger Preview](https://github.com/user-attachments/assets/63ef8f72-5476-4905-8f3e-35a4d9702792)

---

## Overview

Mobile robots equipped with multiple 2D LiDARs — for safety, navigation, or wider field of
view — typically need a single merged scan for downstream algorithms like obstacle avoidance
or SLAM. `scan_2d_merger` solves this by:

1. Subscribing to any number of `LaserScan` topics simultaneously.
2. Synchronising scans across topics using a timestamp-based approximate-time algorithm.
3. Transforming each scan into a common reference frame via TF2.
4. Projecting all rays directly into a shared angular bin array and publishing the result.

The node is implemented as a **composable node** (`rclcpp_components`), so it can run inside
a shared component container with zero-copy intra-process communication, or as a standalone
process depending on your deployment needs.

---

## Features

- **Unlimited input LiDARs** — the custom approximate-time synchronizer imposes no cap on the
  number of input topics. Previous implementations based on `message_filters::ApproximateTime`
  were limited to a fixed number of inputs due to compile-time template constraints; that limitation is gone.

- **Parallel ray projection** — for N > 1 LiDARs, a persistent thread pool (one thread per
  LiDAR) projects all scans simultaneously using `std::barrier` (C++20) for cycle
  synchronization. Threads are created once at startup, eliminating per-callback overhead.

- **No intermediate point cloud** — rays are projected directly from polar coordinates in each
  scanner frame to angular bins in the merged frame. There is no conversion to `PointCloud2`
  and no PCL dependency.

- **Per-topic QoS** — each input topic can independently use `reliable` or `best_effort`
  delivery policy, making it straightforward to mix LiDARs with different publishers.

- **Static and dynamic scanner mounts** — when `moving_frames` is `false` (the default), TF
  transforms are looked up once and cached for the lifetime of the node, which avoids repeated
  TF lookups on every callback. Set `moving_frames: true` for manipulator-mounted or
  otherwise moving scanners.

- **Height filtering** — after transforming each ray into the merged frame, points outside a
  configurable height band `[min_height, max_height]` are discarded. This is useful for
  filtering ground returns or ceiling reflections when LiDARs are tilted.

- **Approximate-time synchronization with two-pass safety** — the synchronizer first finds
  the best candidate scan per topic, verifies that all topics are within the configured
  `sync_slop` window, and only then dequeues any scans. If any topic fails the check, no
  scans are consumed, preventing silent data loss on partial match failures.

- **Composable node** — runs inside `component_container` for efficient multi-node deployments.
  A `MutuallyExclusiveCallbackGroup` ensures serial callback execution regardless of whether
  the container uses a single-threaded or multi-threaded executor.

---

## How It Works

### Synchronization

Each input topic has its own FIFO deque buffer. When a new scan arrives on any topic, the
synchronizer:

1. Computes a reference timestamp as the newest stamp across all buffer heads.
2. Finds the scan closest in time to the reference for each topic (pass 1 — read-only).
3. If every topic has a candidate within `sync_slop` seconds, it dequeues them all (pass 2).
4. If any topic fails, no scans are consumed and the attempt is retried on the next arrival.

### Parallel Projection

For N > 1 inputs, the main thread sets up N work items, then arrives at a `start_barrier`
to release all worker threads simultaneously. Each worker independently projects its assigned
scan's rays into a private `std::vector<float>` range array. When all workers arrive at the
`done_barrier`, the main thread folds the N arrays into the output by taking the minimum
range at each angular bin. The worker threads persist across callbacks, avoiding the cost
of thread creation at LiDAR rates.

For a single input, the projection runs directly on the main thread with no barriers or
workers.

### Ray Projection

For each valid ray (finite range, within `[range_min, range_max]`):

1. Convert polar `(r, θ)` to Cartesian `(x, y, 0)` in the scanner's frame.
2. Apply the TF transform to get `(X, Y, Z)` in the merged frame.
3. Discard if `Z` falls outside `[min_height, max_height]`.
4. Compute 2D range and angle in the merged frame.
5. Round to the nearest angular bin and take the minimum with the current bin value.

Rounding (rather than truncating) is used for the bin index because `float32` precision in
the input scan's `angle_min` and `angle_increment` fields accumulates small errors, and
truncation can shift a reading into the wrong bin.

---

## ROS2 Compatibility

| Distro | Ubuntu | Status |
|---|---|---|
| **Humble** (LTS) | 22.04 | Tested |
| **Iron** | 22.04 | Compatible |
| **Jazzy** (LTS) | 24.04 | Compatible |
| **Kilted Kaiju** | 24.04 | Compatible |

All four distros are supported without code changes. C++20 is required (`std::barrier`,
`std::thread`, `std::atomic`) and is enforced unconditionally in `CMakeLists.txt`.

---

## Prerequisites

### System Requirements

- ROS2 Humble or newer
- C++20-capable compiler (GCC 11+ on Ubuntu 22.04, GCC 13+ on Ubuntu 24.04)
- A configured TF tree providing transforms from each scanner frame to `merged_frame_id`

### ROS2 Dependencies

All dependencies are standard ROS2 packages available via `rosdep`:

| Package | Purpose |
|---|---|
| `rclcpp` | Node API |
| `rclcpp_components` | Composable node registration |
| `sensor_msgs` | `LaserScan` message type |
| `geometry_msgs` | `TransformStamped` |
| `tf2` | Transform data types |
| `tf2_ros` | TF buffer and listener |
| `tf2_geometry_msgs` | `tf2::fromMsg` for Transform conversion |

---

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/ali-pahlevani/2D_Scan_Merger_ROS2.git
```

### 2. Install dependencies

```bash
cd ~/2D_Scan_Merger_ROS2
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build

```bash
colcon build --packages-select scan_2d_merger --symlink-install
```

`--symlink-install` is recommended during development, so that changes to launch files,
config files, and other non-compiled resources take effect without rebuilding.

### 4. Source the workspace

```bash
source ~/2D_Scan_Merger_ROS2/install/setup.bash
```

Add this line to `~/.bashrc` to source it automatically in every new terminal:

```bash
echo "source ~/2D_Scan_Merger_ROS2/install/setup.bash" >> ~/.bashrc
```

---

## Configuration

In the example config file (`param.yaml`), the chosen parameters are for a specific robot. For your own robot, you can adapt the values (based on their definition) to your own use case (default values are listed in the table below as a starting point).

### Parameter Reference

| Parameter | Type | Default | Description |
|---|---|---|---|
| `scan_topics` | `string[]` | `[]` | **Required.** List of input `LaserScan` topic names. Supports 1 to N topics. |
| `scan_policies` | `int64[]` | `[]` | QoS reliability per topic: `0` = reliable, `1` = best effort. Shorter than `scan_topics` defaults missing entries to reliable. |
| `merged_frame_id` | `string` | `"base_link"` | TF frame of the merged output scan. |
| `output_topic` | `string` | `"merged_scan"` | Topic name for the merged `LaserScan`. |
| `sync_slop` | `double` | `0.1` | Maximum timestamp difference [s] for two scans to be considered synchronised. Ignored when only one topic is configured. |
| `queue_size` | `int` | `20` | Per-topic scan buffer depth. Older scans are dropped when the buffer is full. |
| `moving_frames` | `bool` | `false` | Set to `true` if scanner frames move relative to `merged_frame_id` at runtime. When `false`, transforms are looked up once and cached. |
| `tolerance` | `double` | `0.01` | TF lookup timeout [s]. |
| `angle_min` | `double` | `-π` | Start angle of the output scan [rad]. |
| `angle_max` | `double` | `π` | End angle of the output scan [rad]. |
| `angle_increment` | `double` | `π/180` | Angular resolution of the output scan [rad]. Smaller values increase resolution but also message size. |
| `range_min` | `double` | `0.1` | Minimum valid range [m]. Closer readings are discarded. |
| `range_max` | `double` | `float max` | Maximum valid range [m]. Farther readings are discarded. |
| `min_height` | `double` | `-∞` | Minimum point height [m] in `merged_frame_id`. Points below this are discarded. Useful for filtering ground returns. |
| `max_height` | `double` | `+∞` | Maximum point height [m] in `merged_frame_id`. Points above this are discarded. Useful for filtering ceiling reflections. |
| `use_inf` | `bool` | `true` | When `true`, bins with no reading are set to `+inf`. When `false`, they are set to `range_max + inf_epsilon`. |
| `inf_epsilon` | `double` | `1.0` | Offset added to `range_max` for the no-reading fill value when `use_inf` is `false` [m]. |
| `scan_time` | `double` | `1/30` | Nominal scan period [s], written into the output message header. |
| `debug` | `bool` | `false` | Logs each subscribed topic and its QoS policy at startup. |

### Example Configuration File

Create a directory under `config/` named after your robot and copy the example:

```bash
cd ~/2D_Scan_Merger_ROS2/src/scan_2d_merger/config
mkdir my_robot
cp example/param.yaml my_robot/param.yaml
```

Then edit `my_robot/param.yaml` for your setup. For example:

```yaml
scan_topics: ["/lidar_front/scan", "/lidar_rear/scan", "/lidar_left/scan"]
scan_policies: [0, 0, 1]          # front/rear reliable, left best-effort

merged_frame_id: "base_link"
output_topic: "scan/merged"

sync_slop: 0.05                   # 50 ms window for 20 Hz scanners
tolerance: 0.1

min_height: 0.05                  # ignore floor returns
max_height: 1.8                   # ignore ceiling

angle_min: -3.14159
angle_max:  3.14159
angle_increment: 0.00872665       # 0.5 degree resolution

scan_time: 0.05
range_min: 0.15
range_max: 30.0

use_inf: true
queue_size: 20
moving_frames: false
debug: false
```

---

## Running

### Demo (with included rosbag)

The demo launch file plays the bundled rosbag (3 LiDAR scans), starts the merger node, and
opens RViz2 with a preconfigured display:

```bash
ros2 launch scan_2d_merger demo_launcher.launch.py
```

### Custom robot configuration

```bash
ros2 launch scan_2d_merger merger_launcher.launch.py robotname:=my_robot
```

This loads `config/my_robot/param.yaml` and starts the merger inside a composable node
container. Replace `my_robot` with the name of the directory you created under `config/`.

### Directly as a composable node

If you already have a component container running, load the node into it:

```bash
ros2 component load /component_manager_node scan_2d_merger util::LaserScanMerger \
  --param scan_topics:='["/scan0", "/scan1"]' \
  --param merged_frame_id:=base_link
```

### As a standalone node (without a container)

```bash
ros2 run rclcpp_components component_container --ros-args \
  -p scan_topics:='["/scan0", "/scan1"]' \
  -p merged_frame_id:=base_link
```

---

## Topics

### Subscribed

| Topic | Type | Description |
|---|---|---|
| As configured in `scan_topics` | `sensor_msgs/LaserScan` | One subscription per entry in `scan_topics`. QoS set per `scan_policies`. |

### Published

| Topic | Type | QoS | Description |
|---|---|---|---|
| As configured in `output_topic` (default: `merged_scan`) | `sensor_msgs/LaserScan` | Reliable, depth 1 | The merged scan in the `merged_frame_id` frame. |

### TF

The node looks up the transform from each scanner's `header.frame_id` to `merged_frame_id`
using TF2. The `tf_buffer` listens on `/tf` and `/tf_static`. Make sure your robot's TF
tree is broadcasting the required transforms before starting the node.

---

## Testing

The package includes a suite of integration tests that spin the real node with synthetic
scans and a static TF, then verify the merged output numerically.

```bash
colcon test --packages-select scan_2d_merger
colcon test-result --verbose
```

| Test | What it verifies |
|---|---|
| `SingleLidar_IdentityTransform_CorrectBin` | Ray at 0° lands in the correct output bin; all other bins remain `+inf` |
| `SingleLidar_90DegRotation_CorrectBin` | TF rotation correctly maps the ray angle |
| `TwoLidars_SameDirection_MinRangeWins` | Closer reading takes precedence when two LiDARs see the same direction |
| `TwoLidars_ComplementaryFOV_BothHalvesCovered` | Each LiDAR's field contributes to the merged output independently |
| `TwoLidars_SyncSlop_WithinSlop_Merged` | Scans within `sync_slop` are fused and published |
| `TwoLidars_SyncSlop_OutsideSlop_NoOutput` | Out-of-window scans are never published |
| `SingleLidar_RangeFilter_OutOfRangeDropped` | Readings outside `[range_min, range_max]` are discarded |
| `SingleLidar_HeightFilter_HighPointsDropped` | Points above `max_height` after TF transform are discarded |
| `SingleLidar_NanInfRanges_Skipped` | NaN and `+inf` input ranges are silently ignored |

---

## Package Structure

```
scan_2d_merger/
├── config/
│   └── example/
│       └── param.yaml               # Example / template configuration
├── include/
│   └── scan_2d_merger/
│       └── scan_2d_merger.hpp       # Node class declaration
├── launch/
│   ├── merger_launcher.launch.py    # Standard launch (loads config by robot name)
│   └── demo_launcher.launch.py      # Demo launch (plays rosbag + opens RViz2)
├── rviz/
│   └── demo_viz.rviz                # RViz2 config for the demo
├── src/
│   └── scan_2d_merger.cpp           # Node implementation
├── test/
│   ├── laser_bag/                   # Bundled rosbag (3 LiDARs)
│   └── test_scan_merger.cpp         # Integration test suite
├── CMakeLists.txt
├── package.xml
└── LICENSE
```

---

## License

This project is licensed under the **MIT License**. See [LICENSE](LICENSE) for the full text.
