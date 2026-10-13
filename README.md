# safety_navigation — Safety-Aware Navigation Supervisor (ROS 2 Humble)

A custom ROS 2 node that sits **between Nav2 and the robot** and makes sure unsafe velocity
commands never reach the wheels. Nav2 source code is not modified.

Simulation used: **TurtleBot3 (waffle) + Gazebo Classic + Nav2** (`nav2_bringup tb3_simulation_launch.py`).

---

## 1. Prerequisites

Ubuntu 22.04 with ROS 2 Humble (desktop install), then:

```bash
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
                    ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-msgs \
                    ros-humble-gazebo-ros-pkgs ros-humble-launch-testing \
                    ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros \
                    python3-colcon-common-extensions
```

## 2. Build

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
# copy / clone this package so that you have  ~/ros2_ws/src/safety_navigation
cd ~/ros2_ws
colcon build --packages-select safety_navigation
source install/setup.bash
```

## 3. Run

```bash
export TURTLEBOT3_MODEL=waffle          # (the launch file also sets it)
ros2 launch safety_navigation safety_navigation.launch.py
```

Gazebo, RViz and Nav2 start. After about 25 s the launch file publishes the initial pose
automatically (or use **2D Pose Estimate** in RViz). Then send a goal with **Nav2 Goal** in RViz.

Useful launch arguments: `headless:=True`, `use_rviz:=False`, `set_initial_pose:=False`,
`safety_params_file:=/path/to/my.yaml`.

Observe the supervisor:

```bash
ros2 topic echo /safety_state            # NORMAL / CAUTION / EMERGENCY_STOP
ros2 topic echo /safety_debug            # distance, closing speed, thresholds, scale ...
ros2 service call /safety_supervisor/reset std_srvs/srv/Trigger   # recover from E-STOP
```

## 4. Architecture

```
                        +------------------------ safety_supervisor ------------------------+
 Nav2 (controller,  --> | /cmd_vel_nav (Twist)                                               |
 behaviors)             |                                                                    |
                        | /scan  (LaserScan) --> sector distance + approach estimator        |
 Gazebo lidar  -------> | /odom  (Odometry)  --> own speed (braking distance, ego-motion)    |
 Gazebo odom   -------> |                                                                    |
                        |   SafetyCore state machine  (NORMAL / CAUTION / EMERGENCY_STOP)    |
                        |                                                                    |
                        | --> /cmd_vel        (Twist)  safe command, 20 Hz, always published |
                        | --> /safety_state   (String) current state                         |
                        | --> /safety_debug   (String) diagnostics (2 Hz)                    |
                        | <-> ~/reset         (std_srvs/Trigger) controlled recovery         |
                        +--------------------------------------------------------------------+
                                                   |
                                                   v
                                          Gazebo diff-drive plugin (listens to /cmd_vel)
```

**Why the topic names differ from the assignment's example diagram:** the TurtleBot3
Gazebo plugin is hard-wired to `/cmd_vel`, and we must not edit Nav2 or the robot model.
So the launch file re-routes *Nav2's output* to `/cmd_vel_nav` (`SetRemap`), the supervisor
reads that and publishes the safe command on `/cmd_vel`. Topic names are parameters
(`input_cmd_topic`, `output_cmd_topic`), so `cmd_vel_nav → cmd_vel_safe` is a YAML change if the
robot model listens to `/cmd_vel_safe`.

Code layout:

| File | Role |
|---|---|
| `include/.../safety_core.hpp`, `src/safety_core.cpp` | All safety logic, **no ROS dependency** (unit-testable) |
| `src/safety_supervisor_node.cpp` | ROS wrapper: subscriptions, 20 Hz timer, publishers, service, parameters |
| `config/safety_supervisor.yaml` | Every tunable value |
| `launch/safety_navigation.launch.py` | Gazebo + Nav2 + supervisor, remap done for you |
| `launch/supervisor_only.launch.py` | Supervisor alone (for tests / real robot) |
| `models/obstacle_box.sdf`, `scripts/obstacle.sh` | Movable test obstacle for the validation scenarios |
| `test/` | gtest unit tests + launch_testing integration test |

Concurrency: one executor thread; a mutex still guards the core because the parameter callback
and the service also touch it. The output is published from a **fixed-rate timer** (not from
the callbacks) so the watchdogs keep working even when no messages arrive.

QoS: `/scan` and `/odom` use `SensorDataQoS` (best-effort, compatible with both reliable and
best-effort publishers); commands and outputs use reliable depth 10. Timing uses the node clock
(`use_sim_time` in simulation) measured at message *arrival*.

## 5. Safety-state logic

| State | Meaning | Output |
|---|---|---|
| `NORMAL` | Nothing relevant in the direction of travel (or robot idle) | Nav2 command, only clamped to `max_linear_velocity` / `max_angular_velocity` |
| `CAUTION` | Obstacle between effective stop and effective caution distance | Speed reduced progressively |
| `EMERGENCY_STOP` | Critical obstacle, or scan data stale | Zero velocity |

Transitions:

```
          obstacle < caution                      distance <= effective_stop
 NORMAL ----------------------> CAUTION ---------------------------------> EMERGENCY_STOP
   ^  <---------------------------  |                                            |
   |    obstacle > caution+hyst.    |   (also straight NORMAL -> E-STOP)         |
   |                                                                             |
   +---- reset service accepted (path clear >= recovery_clear_time) -------------+
   +---- (sensor fault only) scan healthy >= sensor_recovery_time, auto ---------+
                 both are followed by a speed ramp (recovery_ramp_time)
```

* **Obstacle emergency stop is latched.** Zero output is held even if the obstacle disappears
  and even if Nav2 keeps commanding. Only `~/reset` releases it.
* **Sensor fault** (no scan within `scan_timeout`, or never received): `EMERGENCY_STOP` with reason
  `SENSOR_STALE`. It is *not* latched by default (a fault is not evidence of an obstacle): after the
  scan has been healthy for `sensor_recovery_time` the robot recovers on its own, with a ramp.
  Set `latch_on_sensor_fault: true` to require the reset service here too.
* **Controlled recovery** — `ros2 service call /safety_supervisor/reset std_srvs/srv/Trigger` is rejected unless
  (1) scan data is fresh, (2) the blocked direction has been clear beyond
  `stop_distance + recovery_margin` **continuously** for `recovery_clear_time`. After success, the
  allowed speed ramps from `min_linear_velocity` to the full limit over `recovery_ramp_time`,
  so there is no immediate full-speed restart. The response message tells you why it was rejected.
* **Stale command** (nothing on `/cmd_vel_nav` for `cmd_timeout`): output zero (the Gazebo
  plugin would otherwise keep the last command). State stays `NORMAL`, reason `CMD_STALE`.
* **Stale odometry** (older than `odom_timeout`): not a hazard by itself. The supervisor falls
  back to the *commanded* speed as the ego-speed estimate and keeps working (`odom_ok=0` in the debug topic).

## 6. Obstacle distance and approaching-obstacle logic

**Direction-aware distance**
1. Direction of travel: `0°` if driving forward, `180°` if reversing.
2. If the command also turns, the cone swings toward the turn. For an arc of length
   `caution_distance` the chord direction is half the turned angle: `bias = 0.5·ω·caution_distance/|v|`,
   limited to `max_steer_bias`.
3. The relevant distance is the **minimum valid range inside a cone of `front_sector_angle`** (full width)
   around that direction. Rays that are `inf`, `NaN`, below `range_min` or above `range_max` are ignored.
4. So an obstacle behind a robot driving forward, or beside it, is not even looked at.

**Rotation in place** (`v ≈ 0`, `ω ≠ 0`): a round robot turning on the spot sweeps no new area, so
only **contact-level** obstacles matter: if anything in 360° is closer than `rotation_stop_distance` (0.20 m)
rotation is blocked (angular = 0, state `CAUTION`, reason `ROTATION_BLOCKED`, not latched); otherwise rotation is passed through.

**Progressive speed reduction** (distances are the *effective* ones, see below):

```
scale   = clamp( (d - effective_stop) / (effective_caution - effective_stop), 0, 1 )
v_limit = min_linear_velocity + scale * (max_linear_velocity - min_linear_velocity)
ratio   = min(1, v_limit / |v_cmd|)        # applied to BOTH v and ω -> same curvature
```

**Approaching obstacle** (no object tracking, only the minimum distance of the cone over time):
* Each new scan adds `(time, distance)` to a history of `approach_window` seconds. Closing speed =
  `(oldest − newest distance) / dt`, smoothed by an exponential filter. History is reset when the cone direction changes by more than 20° or nothing is in the cone.
* *Relative* closing speed also contains the robot's own motion. With `compensate_ego_motion: true`
  the ego speed from `/odom` (projected on the cone direction) is subtracted, leaving the **obstacle's own approach speed**. A wall you drive toward gives ≈ 0, a person walking at you gives > 0.
  (Without this, slowing down would shrink the margin again and cause speed hunting.)
* If the obstacle approach speed exceeds `approach_threshold`, both thresholds grow:
  `effective_caution = caution_distance + approach_caution_gain · closing`,
  `effective_stop = stop_distance + approach_stop_gain · closing`.
* Always: `effective_stop += v_ego² / (2·max_deceleration)` (braking distance, uses odom).

Assumptions: the closest return in the cone is "the" obstacle; the lidar is mounted at the robot centre; the
robot is roughly circular; scan rate is ≥ 5 Hz; ego-speed projection uses `cos(cone direction)` (approximation).

## 7. Configuration

All tunables are in `config/safety_supervisor.yaml` and are ROS parameters. Change by editing
the YAML and relaunching, **or live without restart**:

```bash
ros2 param set /safety_supervisor stop_distance 0.6
ros2 param set /safety_supervisor caution_distance 1.5
```

Invalid combinations (e.g. `caution_distance <= stop_distance`) are rejected with a reason.
Integer values for double parameters (`1` instead of `1.0`) are accepted.

## 8. Validation scenarios (what to run and what you should see)

Open 3 terminals (each: `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`).
Terminal A: the launch file. Terminal B: `ros2 topic echo /safety_state`. Terminal C: commands below.
Set the initial pose and send Nav2 a goal straight down the corridor (e.g. 3–4 m ahead in +X from the start).
Helper (`S` = package scripts folder): `S=~/ros2_ws/src/safety_navigation/scripts`.
For a quick look at speed: `ros2 topic echo /cmd_vel --field linear.x`.

| # | Scenario | How | Expected |
|---|---|---|---|
| 1 | Clear path | Send a goal in open space | `NORMAL`, `/cmd_vel` equals Nav2's `/cmd_vel_nav` |
| 2 | Obstacle outside caution | `$S/obstacle.sh spawn box1 3.0 -0.5` (≥ 1 m beyond the robot's path start), robot far away | still `NORMAL` |
| 3 | Obstacle enters caution | Drive toward a box; or `spawn box1` ~0.9 m in front of the robot | `CAUTION`, `linear.x` shrinks smoothly as distance falls |
| 4 | Obstacle in critical region | Box ≤ 0.45 m ahead (or keep driving) | `EMERGENCY_STOP`, `/cmd_vel` = 0 |
| 5 | Obstacle behind | Spawn a box 0.4 m **behind** the robot (≈ x−0.4 of robot) while it drives forward | stays `NORMAL`, no stop |
| 6 | Sensor stale | `$S/obstacle.sh scan_off` | `EMERGENCY_STOP` (`SENSOR_STALE`) within `scan_timeout` |
| 7 | Sensor valid again | `$S/obstacle.sh scan_on` | after ~1 s auto-recovers, speed ramps up (E-STOP before: needs reset only if `latch_on_sensor_fault`) |
| 8 | Approaching obstacle | `spawn box2` ahead, then `$S/obstacle.sh move box2 -0.4 0` toward the robot | `CAUTION` starts earlier than 1.0 m and the stop comes at a larger distance (`/safety_debug`: `obstacle_closing > 0`, bigger `eff_stop`) |
| 9 | Parameters changed | `ros2 param set /safety_supervisor caution_distance 2.0` (or `stop_distance`) | thresholds change immediately; no rebuild |
| 10 | Nav2 keeps commanding in E-STOP | Compare `ros2 topic echo /cmd_vel_nav` (non-zero) with `/cmd_vel` (zero) | only zero reaches the robot |

Recovery after an obstacle stop: remove the obstacle (`$S/obstacle.sh delete box1`), wait ≥ 1 s, then
`ros2 service call /safety_supervisor/reset std_srvs/srv/Trigger` → `success: True`. Calling it too early returns
`success: False` and a message explaining why.

## 9. Automated tests

```bash
cd ~/ros2_ws
colcon test --packages-select safety_navigation --event-handlers console_direct+
colcon test-result --verbose
```

* `test/test_safety_core.cpp` (gtest, 22 tests): clear path, outside caution, velocity clamping, progressive reduction,
  curvature preserved, latched emergency stop, obstacle behind (forward vs reverse), side obstacle, turn-biased cone, rotation,
  stale scan + hold-off + ramp, no scan ever, stale command, stale odometry, latch-on-fault mode, reset rejection / acceptance / ramp, approaching
  obstacle vs static obstacle, parameter change, invalid parameters.
* `test/test_supervisor_integration.py` (launch_testing): starts the real node and checks over real topics: normal forwarding, emergency
  stop that survives Nav2 flooding commands, rejected + accepted reset service, stale sensor stop + auto recovery.

## 10. Known limitations / future work

* Only a 2D lidar is used; obstacles above/below the scan plane are invisible. No costmap or depth camera fusion.
* Cone uses the closest point only: no clustering/tracking, so two objects crossing can reset the closing-speed estimate.
* Obstacle emergency stops are latched and a robot wedged in front of a wall cannot back out by itself (the reset is rejected while the
  wall is within `stop_distance + recovery_margin`). Improvement: an "escape mode" that allows slow motion *away* from the obstacle.
* Diff-drive only (no lateral velocity). Footprint is treated as a circle; a polygon footprint check would be more precise.
* Rotation check is contact-level only. Fine for round robots, not for long ones.
* Parameters like `stop_distance` must be tuned to the robot: too large and Nav2 will stop in narrow passages.
* Timing uses message arrival time, not header stamps (robust, but ignores sensor latency).
* Improvements with more time: lifecycle node, `diagnostic_msgs`, custom state message with reason and distances, Nav2 collision_monitor comparison, rosbag-based replay tests.

## 11. Demo video checklist (2–4 minutes, screen recording)

1. Launch + show `ros2 topic echo /safety_state` and `/cmd_vel` side by side with RViz/Gazebo (scenario 1).
2. Spawn obstacle → CAUTION speed shrinking (3), then E-STOP (4); show `/cmd_vel_nav` non-zero vs `/cmd_vel` zero (10).
3. Reset service rejected, then accepted after clearing (recovery).
4. Obstacle behind (5), `scan_off` / `scan_on` (6, 7), moving box (8), `ros2 param set` (9).
5. Run `colcon test` at the end and show the passing results.

## 12. Troubleshooting

* Robot does not move at all: the supervisor starts in `EMERGENCY_STOP / SENSOR_STALE` until `/scan` arrives (~1 s). Check `ros2 topic hz /scan`.
* Robot ignores the supervisor: make sure only the supervisor publishes `/cmd_vel` (`ros2 topic info /cmd_vel -v` should show one publisher) and Nav2 publishes `/cmd_vel_nav`.
* Gazebo models missing: `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models`.
* Obstacle box does not move: check `ros2 topic list | grep box1` shows `/box1/cmd_vel`.
* Frequent false stops in narrow places: reduce `stop_distance` or `front_sector_angle`.
