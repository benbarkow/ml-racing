# Real-car setup (sim-to-real)

Lab notes for running the ROS stack on the 1:10 F1TENTH car (Jetson Nano + VESC)
with a Vicon motion-capture system providing the car's pose.
Translated and cleaned up from the original German notes.

## Components

| Where | What | Source |
|---|---|---|
| Car (Jetson Nano) | ROS 1 Melodic + [f1tenth_system](https://github.com/f1tenth/f1tenth_system) drivers (VESC, teleop) | [`ros/car_init`](../ros/car_init) |
| PC or car | Vicon `vrpn_client_ros` (ROS 1) + `ros1_bridge` to ROS 2 | [`ros/bridge`](../ros/bridge) |
| PC or car | ROS 2 Humble workspace: camera publisher, tracking node, policy node | [`ros/ros2_ws`](../ros/ros2_ws) |

## Startup sequence

1. **Car drivers** (on the Jetson):
   ```bash
   cd ros/car_init && ./launch.sh          # builds and starts the ros1-melodic-f1tenth container
   roslaunch racecar teleop.launch         # inside the container
   ```
2. **Vicon bridge** (ROS 1 → ROS 2):
   ```bash
   cd ros/bridge && sudo docker compose up
   ```
3. **ROS 2 workspace**:
   ```bash
   cd ros/ros2_ws && ./launch.sh           # builds the image and opens a shell in it
   colcon build && source install/setup.bash
   ros2 topic list                         # the Vicon topics below should be visible
   ```

## Nodes on the car (ROS 2)

| Node | Package | What it does |
|---|---|---|
| `cam_features_pub` | `car_camera` | Reads the onboard camera, keeps a stack of the last 3 frames (80×60), runs the perception CNN (`stack_cnn_v2.onnx`) and publishes the 8 state features on `car_video_features` |
| `net` | `car_controller` | Runs the driving policy (`track_drive_cnn_44.onnx`) on those features and sends discrete steering / speed commands to the `action` service |
| `car_controller` | `car_controller` | Serves `action` and forwards commands to the VESC topics below |
| `logging` | `car_controller` | Records Vicon pose, CNN features and actions at 20 Hz to `logs/*.json` |

## Topics

Actuation (ROS 1, on the car), type `std_msgs/Float64`:

| Purpose | Topic |
|---|---|
| Speed | `/vesc/commands/motor/unsmoothed_speed` |
| Steering | `/vesc/commands/servo/unsmoothed_position` |

Vicon tracking (bridged to ROS 2), for the rigid body `BaCar`:

| Topic | Content |
|---|---|
| `/vrpn_client_node/BaCar/pose` | position (x, y, z) and orientation quaternion (x, y, z, w) |
| `/vrpn_client_node/BaCar/twist` | linear / angular velocity |
| `/vrpn_client_node/BaCar/accel` | linear / angular acceleration |

Camera (ROS 2): `car_video_frames`, an 80×60 grayscale frame flattened into a
`std_msgs/Float32MultiArray`, published at ~30 Hz by `car_camera camera_pub`.

### Coordinate frames

| Vicon | Meaning | Unity |
|---|---|---|
| x | left → right | x |
| y | back → front | z |
| z | up / down | y |

## Networking

The car's ROS 1 master runs at `192.168.0.104:11311` on the lab Wi-Fi.
For the bridge on a PC to reach it:

- add `192.168.0.104  JetsonNano` to `/etc/hosts` on the PC
- set `ROS_MASTER_URI=http://JetsonNano:11311` (or the IP) and `ROS_IP` to the PC's own address in
  [`ros/bridge/docker-compose.yaml`](../ros/bridge/docker-compose.yaml)

Troubleshooting log from the lab: with the bridge on the PC and the car's topics active, the
ROS 1 topics were visible on the car but nothing arrived at the bridge or on the PC's ROS 2 side.
The `/etc/hosts` entry and `ROS_MASTER_URI` setting above are the configuration applied in
response.
