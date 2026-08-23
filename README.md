# Vision LiDAR Object Following - Gazebo & ROS2 (All from scratch)
, except for the human model :p
Demo Video (click it):
[![Watch the video](https://img.youtube.com/vi/uNYvJtAvjr0/maxresdefault.jpg)](https://youtu.be/uNYvJtAvjr0)

## Things to run
Simulation → Top-level bring-up:
```
ros2 launch bringup bringup.launch.py
```
This composes the following launch files: `facility_world.launch.py` → `following_bot.launch.py` → `bot_vision.launch.py` → `obstacle_avoidance.launch.py` → RViz2. `use_sim_time` defaults to `true` here and is threaded through to `following_bot`, `bot_vision`, and `obstacle_avoidance` via `launch_arguments`.
If you wish to launch the packages one by one, use the individual commands shown above. (example: ros2 launch facility_world facility_world.launch.py)

`bot_vision.launch.py` and `obstacle_avoidance.launch.py` default `use_sim_time` to `false` when run standalone — only `bringup.launch.py` flips it to `true`, so running vision/avoidance nodes in isolation against a sim clock requires passing `use_sim_time:=true` explicitly.

`bot_vision.launch.py` has an explicit ordering dependency documented inline: the camera stream (`/following_bot/camera/rgb`) is published by the Gazebo bridge in `following_bot`'s own launch file, and is expected to already be running before `bot_vision` nodes start.

`facility_world.launch.py` sets `GZ_SIM_RESOURCE_PATH` to the package's `models` directory so the custom SDF models (bins, bottles, can, shelf) resolve, and launches `gz sim -r` directly on `worlds/facility.world` rather than through `ros_gz_sim`'s gz_sim launch wrapper.

Keyboard Teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true`


## Vision pipeline (bot_vision)

Four nodes launched together by `bot_vision.launch.py`, all sharing `config/bot_vision_params.yaml`. Pipeline is as follows!

```
/following_bot/camera/rgb
  → yolo_detector          (yolov8n.pt, confidence_threshold=0.4, rate_hz=30)
      → /bot_vision/detections, /bot_vision/detected_classes
  → target_selector        (target_class="person", mode="class_and_proximity", min_confidence=0.5)
      → /bot_vision/selected_detection, /bot_vision/target_id
  → tracker_node           (KCF tracker, camera_hfov_rad=1.047, known_object_width_m=0.15,
                             reacquire_iou_threshold=0.3, control_rate_hz=30)
      → bot_vision/target_bearing_distance, /odom/target_detected
  → follow_controller      (desired_distance=1.0m, linear_gain=0.6, angular_gain=1.8,
                             max_linear_speed=0.5, max_angular_speed=1.5,
                             distance_deadband=0.1, bearing_deadband=0.02,
                             target_timeout_sec=0.5, frame_id="base_link")
      → /odom/desired_cmd_vel
```

`yolo_detector`'s `model_path` is set to the bare filename `"yolov8n.pt"` btw — not a package share path; if it silently fails to load a model, check where the node was launched from.

## Obstacle Avoidance State Machine (obstacle_avoidance)

This node implements a reactive safety layer using a simple state machine. The robot has four main "moods" that determine how it behaves in response to its vision system and LiDAR sensor.

---

### The Four States

#### PATROL – “Explore on my own”

- The robot drives forward, occasionally turning left or right at random.
- It does not rely on the vision system for motion.
- It keeps patrolling until the vision system reports a valid target.

---

#### TRACK – “Follow the object”

- The robot stops making its own decisions.
- It simply forwards the speed.
- **Safety rule:** It will only move if the target is still confirmed **and** the upstream commands are fresh. Otherwise, it stays still.

---

#### SEARCH – “I lost it - look around”

- The robot spins in place (rotates) to try and re‑acquire the target.
- It never drives forward in this state because no target is currently confirmed.
- **Two possible exits:**
  - **Target found** → go back to TRACK.
  - **Search times out** → give up and return to PATROL.

---

#### AVOID – “Danger - get out of the way!”

- If the front LiDAR detects an obstacle too close, AVOID instantly takes over.
- The robot stops driving forward and turns away from the obstacle (choosing the side with more open space).
- It remembers which state it interrupted (PATROL, TRACK, but not SEARCH).
- Once the front path is clear for a short moment, it hands control back to that previous state and resumes what it was doing.

---

### The Ultimate Safety Net

#### Emergency Stop
- If an obstacle gets **extremely** close (within a very short distance), the robot slams the brakes immediately.
- This overrides **everything**, including the AVOID state, to prevent a collision at all costs.

---

### How They Flow Together (The Big Picture)

| Starting State | Event | Next State |
| :--- | :--- | :--- |
| PATROL | Vision detects a target | TRACK |
| TRACK | Vision loses the target | SEARCH |
| SEARCH | Target re‑acquired | TRACK |
| SEARCH | Search timeout | PATROL |
| *Any state* | Obstacle appears too close | AVOID |
| AVOID | Path is clear for a while | *Previous state (PATROL/TRACK)* |
| *Any state* | Obstacle is dangerously close | *Emergency Stop (zero speed)* |

---
