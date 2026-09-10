# FlagShip

ROS2 package that bridges the [DroidControlShip](https://github.com/BlueAndi/DroidControlShip) to the ROS2 ecosystem. It receives vehicle telemetry and publishes standard ROS2 messages, enabling SLAM and autonomous navigation for the [Pololu Zumo](https://www.pololu.com/category/170/zumo-32u4-oled-robot) robot running [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) and [DroidControlShip](https://github.com/BlueAndi/DroidControlShip). It also provides ready-to-use configuration for `robot_localization`, `slam_toolbox`, and Nav2.

> **Note:** Currently only Webots simulation is supported. Hardware target support is not yet implemented.

## System Context

```
RadonUlzer (Zumo robot)
        │  SerialMuxProt
        ▼
DroidControlShip (ZumoComSystem shield)
        │  JSON over TCP or Serial
        ▼
FlagShip (this package — ROS2)
        │  ROS2 topics
        ▼
  SLAM / Nav2 / RViz
```

## ROS2 Interface

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Wheel odometry from the robot |
| `/imu` | `sensor_msgs/Imu` | IMU data (angular velocity Z, linear acceleration X) |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint positions and velocities |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan (angle-corrected passthrough from `scan_raw`) |

### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity commands forwarded to the robot over TCP |
| `/scan_raw` | `sensor_msgs/LaserScan` | Raw LiDAR input; republished as `scan` with corrected orientation |

### Parameters (`tcp_bridge` node)

| Parameter | Default | Description |
|---|---|---|
| `host` | `0.0.0.0` | TCP listen address |
| `port` | `8888` | TCP listen port |
| `wheel_radius` | `0.018` m | Wheel radius for joint state calculation |
| `wheel_separation` | `0.075` m | Wheel separation for angular velocity calculation |

## Installation

### 1. Install RadonUlzer and DroidControlShip in WSL

Clone and set up both companion projects inside your WSL environment:

- [RadonUlzer](https://github.com/BlueAndi/RadonUlzer) — robot firmware
- [DroidControlShip](https://github.com/BlueAndi/DroidControlShip) — ZumoComSystem shield firmware

### 2. Set up WSL, ROS2 Jazzy, Webots and VSCode

Follow steps 1–4 of the [DroidControlShip ROS2 setup guide](https://github.com/BlueAndi/DroidControlShip/blob/develop/doc/ROS2/README.md#installation-steps):

| Step | Guide |
|---|---|
| 1 | [WSL with Ubuntu](https://github.com/BlueAndi/DroidControlShip/blob/develop/doc/ROS2/setup/wsl.md) |
| 2 | [ROS2 Jazzy](https://github.com/BlueAndi/DroidControlShip/blob/develop/doc/ROS2/setup/ROS2_Jazzy.md) |
| 3 | [Webots](https://github.com/BlueAndi/DroidControlShip/blob/develop/doc/ROS2/setup/Webots.md) |
| 4 | [VSCode in WSL](https://github.com/BlueAndi/DroidControlShip/blob/develop/doc/ROS2/setup/vscode.md) |

### 3. Install ROS2 packages

```bash
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-webots-ros2
```

## Build

Build all packages on the first run:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

After the first build, only FlagShip needs to be rebuilt when making changes:

```bash
cd ~/ros2_ws
colcon build --packages-select flagship
source install/setup.bash
```

## Run

Before launching FlagShip, ensure the following are already running:

- **Webots** — simulation open with a compatible world/map
- **RadonUlzer** — running the *RemoteControl* application
- **DroidControlShip** — running the *Slam* application,

### SLAM Mode

Each ros2 launch requires a separate terminal. Source the workspace in each one before running:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Launch the bridge node:

```bash
ros2 launch flagship bridge_lidar_launch.py
```

Launch SLAM Toolbox:

```bash
ros2 launch flagship slam_toolbox_launch.py
```

Control the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel -p stamped:=true -p speed:=0.2 -p turn:=0.2
```

Launch RViz:

```bash
rviz2
```

In RViz, open the config file `src/FlagShip/rviz/defaultConfig.rviz`.

Drive the robot around, using the teleop keyboard, to build the map. Once done, save it via the SLAM Toolbox panel in RViz:

1. In RViz, add the **SlamToolboxPlugin** panel.
2. Choose a save method:
   - **Serialize** — saves the full pose-graph for later use with SLAM Toolbox.
   - **Save Map** — exports a standard `.pgm`/`.yaml` map for use with Nav2 or other tools.

### Navigation Mode

Each ros2 launch requires a separate terminal. Source the workspace in each one before running:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Launch the bridge node:

```bash
ros2 launch flagship bridge_lidar_launch.py
```

Launch Nav2 with a previously saved map:

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=<path to map.yaml> \
  params_file:=<path to nav2_params.yaml> \
  use_sim_time:=true
```

Launch RViz:

```bash
rviz2
```

In RViz:

1. Set the robot's initial pose using the **2D Pose Estimate** tool.
2. Send a navigation goal using the **2D Goal Pose** tool.

The robot will autonomously navigate to the goal.