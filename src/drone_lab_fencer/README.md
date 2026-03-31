# drone\_lab\_fencer

A **ROS 2 Humble** (C++) **middleman geofencing** package for a multi-drone
indoor laboratory equipped with OptiTrack motion capture.

The middleman PC receives student position commands, validates them against a
configurable 3-D bounding box loaded from YAML, and only then forwards them
to the actual MAVROS / ArduPilot stack.  Commands that fall outside the fence
are silently dropped and the drone is instructed to hold its current position.

---

## Architecture

```
Student PC ──► /student/droneXX/setpoint_raw/local
                        │
                   ┌────▼────┐
                   │ Fencer  │  ← checks bounding box
                   └────┬────┘
                        │  (pass / hold)
                        ▼
         /droneXX/mavros/setpoint_raw/local  ──► MAVROS ──► Drone
```

---

## 1. Network Layout

| Entity | IP | Notes |
|---|---|---|
| Middleman PC | `192.168.18.201` | ROS 2 DDS host |
| Drone SysID 10 | `192.168.18.110` | MAVLink → UDP `14510` |
| Drone SysID 11 | `192.168.18.111` | MAVLink → UDP `14511` |
| … | … | … |
| Drone SysID 19 | `192.168.18.119` | MAVLink → UDP `14519` |

Each drone's companion computer / autopilot must send its MAVLink stream to
`192.168.18.201` on port **14500 + SysID**.

---

## 2. Installation

```bash
# Clone into your colcon workspace
cd ~/colcon_ws/src
cp -r /path/to/drone_lab_fencer .

# Build
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_lab_fencer
```

Both `safety_fencer` and `takeoff_service` are C++ executables compiled
automatically by `colcon build`.  No additional steps are needed.

---

## 3. Launching the System (Middleman PC)

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch drone_lab_fencer lab_bringup.launch
```

This starts:

* **10 MAVROS nodes** (`/drone10/mavros` … `/drone19/mavros`), each
  listening on its UDP port.
* **safety\_fencer** — the geofencing gate (parameters loaded from YAML).
* **takeoff\_service** — safe arm-and-takeoff helper.

### Customising the Fence

Edit `config/fence_params.yaml` **before** building/launching.  The YAML
uses the standard ROS 2 parameter file format:

```yaml
safety_fencer:
  ros__parameters:
    fence:
      x_min: -3.0
      x_max:  3.0
      y_min: -3.0
      y_max:  3.0
      z_min:  0.1
      z_max:  3.0
    drone_id_min: 10
    drone_id_max: 19
```

Or override at runtime:

```bash
ros2 run drone_lab_fencer safety_fencer \
    --ros-args -p fence.x_max:=3.0 -p fence.y_max:=3.0
```

---

## 4. Student Setup

### 4.1 Network Configuration

ROS 2 uses DDS for discovery.  Students on the same LAN automatically
discover topics if they share the same `ROS_DOMAIN_ID` (default `0`).

On every student PC:

```bash
export ROS_DOMAIN_ID=0              # must match the middleman
```

> **Tip:** Add this to `~/.bashrc` so it persists across terminals.

Verify connectivity:

```bash
ros2 topic list          # Should show /drone10/mavros/… through /drone19/mavros/…
```

### 4.2 Sending Position Commands

Publish a `mavros_msgs/msg/PositionTarget` to the **student** topic.  The
fencer will validate and forward it.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget

class StudentCommander(Node):
    def __init__(self):
        super().__init__('student_commander')
        # Replace 10 with your assigned drone ID
        self.pub = self.create_publisher(
            PositionTarget,
            '/student/drone10/setpoint_raw/local',
            1)
        self.timer = self.create_timer(1.0 / 20.0, self.send_cmd)  # 20 Hz

    def send_cmd(self):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE)
        msg.position.x = 0.0   # metres
        msg.position.y = 0.0
        msg.position.z = 1.0   # altitude
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(StudentCommander())
```

### 4.3 Arming & Takeoff

Instead of calling MAVROS arming/mode services directly, use the safe
wrapper:

```bash
ros2 service call /student/drone10/takeoff std_srvs/srv/Trigger "{}"
```

The service will:

1. Check that OptiTrack data is flowing.
2. Stream a position setpoint to satisfy the OFFBOARD/GUIDED pre-arm counter.
3. Switch the FCU to GUIDED (ArduPilot) or OFFBOARD (PX4) mode.
4. Arm the vehicle.
5. Return `success: true` once the drone is climbing.

---

## 5. Topic & Service Reference

| Purpose | Topic / Service | Type |
|---|---|---|
| Student command (input) | `/student/droneXX/setpoint_raw/local` | `mavros_msgs/msg/PositionTarget` |
| Fenced command (output) | `/droneXX/mavros/setpoint_raw/local` | `mavros_msgs/msg/PositionTarget` |
| Drone pose | `/droneXX/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` |
| Safe takeoff | `/student/droneXX/takeoff` | `std_srvs/srv/Trigger` |
| MAVROS state | `/droneXX/mavros/state` | `mavros_msgs/msg/State` |

*(Replace `XX` with the drone's SysID: 10 – 19.)*

---

## 6. Tuning & Tips

* **Fence values** are in the MAVROS local frame (usually ENU). Make sure
  your OptiTrack → MAVROS bridge publishes in the same frame.
* The fencer **drops** out-of-bounds commands and re-publishes the drone's
  current position, effectively commanding a hover-hold.
* For smooth flight, students should publish setpoints at **≥ 20 Hz**.
* Monitor rejected commands in real-time:
  ```bash
  ros2 topic echo /rosout | grep BLOCKED
  ```

---

## 7. File Listing

```
drone_lab_fencer/
├── CMakeLists.txt                     # ament_cmake build
├── package.xml                        # ROS 2 package manifest
├── README.md
├── config/
│   └── fence_params.yaml              # ROS 2 parameter YAML (fence bounds)
├── launch/
│   └── lab_bringup.launch             # ROS 2 XML launch file
└── src/
    ├── safety_fencer.cpp              # Geofencing gate (rclcpp)
    └── takeoff_service.cpp            # Safe arm/takeoff service (rclcpp)
```

### YAML Fence Configuration

The fence parameters are defined in `config/fence_params.yaml` using the
standard ROS 2 parameter file format (`node_name: ros__parameters: ...`).
Both C++ nodes read them with `node->declare_parameter()` /
`node->get_parameter()`.  The launch file loads the YAML via
`<param from="..."/>`.

---

## License

MIT
