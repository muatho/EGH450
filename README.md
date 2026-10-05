# UAV Guidance, Target Detection and Payload Deployment

ROS Noetic guidance software for an autonomous UAV survey mission. The system follows a lawnmower search pattern, uses Breadcrumb for path planning, responds to detected targets, deploys the corresponding payload, and supports landing at a selected ArUco marker.

The project supports a real UAV through MAVROS and an emulator through `uavasr_emulator`.

> These instructions describe the project configuration used during development. Replace the repository URL below and use the appropriate launch files for your flight controller, localisation, camera, and Breadcrumb setup.

## Features

- Lawnmower survey waypoints for an 8 × 5 m search area.
- Breadcrumb path planning between survey waypoints.
- Person and backpack detection confirmation.
- Target-dependent payload selection:
  - Class `0`: person → servo A.
  - Class `1`: backpack → servo B.
- Payload deployment sequence: approach, descend, release, climb, and resume.
- ArUco marker collection and manual landing-marker selection.
- Mission abort and landing in place at battery levels of 20% or below.
- Fresh-position checks before mission startup and payload release.
- Optional waypoint and live-position boundary checks.
- Emulator battery-drain simulation for low-battery testing.

## Requirements

The development system used:

- Ubuntu 20.04.
- ROS Noetic.
- Python 3 and a C++ compiler.
- A Catkin workspace.
- MAVROS for communication with the real flight controller.
- The project’s `spar_node`, `spar_msgs`, Breadcrumb, and emulator packages.
- `vision_msgs` for target observations.
- A Raspberry Pi with `RPi.GPIO` for physical servo control.

ROS Noetic is a legacy distribution. Use the project’s matching environment rather than assuming compatibility with ROS 2 or a newer Ubuntu release.

## Installation

### 1. Install ROS Noetic

Follow the [ROS Noetic Ubuntu installation instructions](https://wiki.ros.org/noetic/Installation/Ubuntu).

After installation, source ROS:

```bash
source /opt/ros/noetic/setup.bash
```

### 2. Install development tools and ROS dependencies

With the ROS package repository configured:

```bash
sudo apt update
sudo apt install \
    build-essential \
    git \
    python3-rosdep \
    python3-yaml \
    ros-noetic-catkin \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-vision-msgs
```

On the Raspberry Pi used for physical payload control:

```bash
sudo apt install python3-rpi.gpio
```

Complete any additional MAVROS setup required by your vehicle configuration, including the GeographicLib datasets described in the [MAVROS installation documentation](https://github.com/mavlink/mavros/tree/master/mavros#installation).

### 3. Clone the repository

The following assumes the repository contains ROS packages and should be placed inside the workspace’s `src` directory.

```bash
mkdir -p ~/catkin_ws_G6/src
cd ~/catkin_ws_G6/src

git clone --recurse-submodules <YOUR_GITHUB_REPOSITORY_URL> uav_project
```

Replace `<YOUR_GITHUB_REPOSITORY_URL>` before running the command.

If the repository contains a complete Catkin workspace, use its existing workspace structure instead of nesting another workspace inside `src`.

### 4. Resolve package dependencies

If `rosdep` has not been initialised on the machine:

```bash
sudo rosdep init
```

Then run:

```bash
rosdep update

cd ~/catkin_ws_G6
rosdep install --from-paths src --ignore-src -r -y
```

Custom packages such as Breadcrumb and `spar_msgs` must also be present in the workspace or an already-sourced dependency workspace. `rosdep` does not automatically fetch missing private or custom source repositories.

### 5. Build the workspace

```bash
cd ~/catkin_ws_G6
source /opt/ros/noetic/setup.bash

catkin_make -j2
source devel/setup.bash
```

Source the environment in every terminal used for the project:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws_G6/devel/setup.bash
```

### 6. Verify package discovery

```bash
rospack find spar_node
rospack find spar_msgs
rospack find breadcrumb
rospack find uavasr_emulator
```

If a package cannot be found, check that it exists in the workspace, the build succeeded, and the workspace has been sourced.

## Main Components

| Component | Purpose |
|---|---|
| `test_breadcrumb_camera` | Survey mission, target diversion, payload selection, battery monitoring, and landing workflow |
| `nav_mission.launch` | Guidance parameters and conditional startup of the physical payload controller |
| `payload-v1.2.py` | Receives actuator commands and drives the Raspberry Pi servo outputs |
| SPAR | Executes flight-motion action goals |
| Breadcrumb | Supplies paths between requested positions |
| Object position estimator | Publishes target observations with map-frame positions |
| `uavasr_emulator` | Simulated vehicle state, movement, position, and battery |

## ROS Interfaces

| Topic or service | Type | Purpose |
|---|---|---|
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Real-UAV position |
| `/uavasr/odom` | `nav_msgs/Odometry` | Emulator position |
| `/mavros/battery` | `sensor_msgs/BatteryState` | Real-UAV battery |
| `/uavasr/battery` | `sensor_msgs/BatteryState` | Emulator battery |
| `/objects/observations` | `vision_msgs/Detection3DArray` | Target classes, confidence scores, and positions |
| `/world_model/aruco_markers` | `visualization_msgs/MarkerArray` | Discovered landing markers |
| `/actuator_control/actuator_a` | `std_msgs/Bool` | Servo A command |
| `/actuator_control/actuator_b` | `std_msgs/Bool` | Servo B command |
| `/breadcrumb/request_path` | `breadcrumb/RequestPath` | Path-planning service |
| `/spar/flight` | `spar_msgs/FlightMotionAction` | Flight-motion action interface |

Target observations must use the `map` frame. Position sources, target coordinates, waypoints, and planning data must use compatible coordinate systems.

## Starting the System

Start the supporting stack before guidance:

1. ROS master.
2. Real vehicle interface or emulator.
3. SPAR flight-action server.
4. Breadcrumb planner.
5. Localisation and perception components as required.
6. Guidance.

Use the project’s existing launch files for these supporting components. `nav_mission.launch` does not start the entire flight and perception stack.

Check that the dependencies are available:

```bash
rosnode list
rosservice list | grep breadcrumb
rostopic list | grep spar/flight
```

Starting guidance can send flight-motion goals immediately once its startup checks pass. Start it only when the vehicle or emulator is ready.

### Real UAV

With MAVROS, localisation, SPAR, Breadcrumb, and the required perception nodes already running:

```bash
roslaunch spar_node nav_mission.launch \
    pose_source:=real \
    deploy:=false
```

This runs the survey with target diversion and payload deployment disabled.

For an authorised payload mission:

```bash
roslaunch spar_node nav_mission.launch \
    pose_source:=real \
    deploy:=true
```

The default real-UAV inputs are:

```text
Position: /mavros/local_position/pose
Battery:  /mavros/battery
```

### Emulator: survey without payload deployment

With the emulator, SPAR, and Breadcrumb running:

```bash
roslaunch spar_node nav_mission.launch \
    pose_source:=emulator \
    deploy:=false
```

The default emulator inputs are:

```text
Position: /uavasr/odom
Battery:  /uavasr/battery
```

> `deploy:=false` disables target diversion as well as payload release.

### Emulator: full target-diversion sequence

To run guidance without having `nav_mission.launch` automatically start the GPIO payload controller:

```bash
rosrun spar_node test_breadcrumb_camera \
    _pose_source:=emulator \
    _deploy:=true
```

This still publishes actuator commands. For a simulation-only test, verify that no physical servo controller is subscribed:

```bash
rostopic info /actuator_control/actuator_a
rostopic info /actuator_control/actuator_b
```

Use topic monitoring to observe simulated release commands:

```bash
rostopic echo /actuator_control/actuator_a
```

```bash
rostopic echo /actuator_control/actuator_b
```

For a physical payload bench test, the servo controller must be running and the mechanism must be clear.

## Mission Sequence

1. Validate the waypoint list.
2. Wait for Breadcrumb and SPAR.
3. Obtain a valid battery reading.
4. Subscribe to the selected position topic and wait for a fresh position.
5. Move to the initial survey waypoint.
6. Follow the lawnmower survey, requesting Breadcrumb paths between waypoints.
7. If an eligible target is confirmed:
   - Interrupt the survey.
   - Fly above the target at survey altitude.
   - Descend to 0.8 m in the navigation coordinate frame.
   - Wait five seconds.
   - Recheck deployment permission, battery, and position freshness.
   - Command the target’s assigned servo.
   - Climb back to survey altitude.
   - Return to the interruption position.
   - Resume the survey.
8. After the survey, wait for an operator to select a detected ArUco ID.
9. Travel to the selected marker, approach at landing altitude, and command landing.

The 0.8 m deployment altitude is an absolute coordinate-frame height, not a measured clearance above the detected object.

## Target Confirmation and Payload Mapping

| Detection ID | Target | Actuator |
|---|---|---|
| `0` | Person | Servo A |
| `1` | Backpack | Servo B |

Default confirmation settings include:

- Confidence threshold: 0.60.
- Three nearby observations.
- Confirmation radius: 0.75 m.
- Repeat-service suppression radius: 1.0 m.

The current implementation uses the `backpack_confirmations_required` setting for confirmation of both supported classes.

Each payload is commanded at most once per guidance process. Restarting guidance resets these software flags; it does not physically reload or close the mechanism.

## Survey Waypoints

The current lawnmower proposal covers a rectangle with:

```text
X: −4.0 to 4.0 m
Y: −3.0 to 2.0 m
```

At the default survey altitude of 2.0 m:

```python
waypoints = [
    [ 0.0,  0.0, surveyAltitude, 0.0],

    [-3.5, -2.5, surveyAltitude, 0.0],
    [ 3.5, -2.5, surveyAltitude, 0.0],

    [ 3.5, -1.5, surveyAltitude, 0.0],
    [-3.5, -1.5, surveyAltitude, 0.0],

    [-3.5, -0.5, surveyAltitude, 0.0],
    [ 3.5, -0.5, surveyAltitude, 0.0],

    [ 3.5,  0.5, surveyAltitude, 0.0],
    [-3.5,  0.5, surveyAltitude, 0.0],

    [-3.5,  1.5, surveyAltitude, 0.0],
    [ 3.5,  1.5, surveyAltitude, 0.0],
]
```

Rows are 1 m apart, and their endpoints are 0.5 m inside the search-area edges.

Full camera coverage assumes the actual footprint is sufficiently large and correctly oriented. The displayed 1.4 × 2.3 m footprint is a visualisation assumption; obstacle detours and target diversions can leave coverage gaps.

## Position Checks

Guidance uses an explicit position topic:

- Real mode: `/mavros/local_position/pose`.
- Emulator mode: `/uavasr/odom`.
- Override through the `pose_topic` launch argument or private parameter.

Before starting, guidance waits up to 10 seconds for a valid, fresh position.

Before payload release, position data must be no more than one second old. Missing, stale, or invalid position data blocks release; missing position alone does not command landing.

These checks are not a complete in-flight localisation-loss failsafe.

## Battery Behaviour

### Real UAV

Guidance reads `sensor_msgs/BatteryState.percentage` from `/mavros/battery`.

The value is a fraction:

```text
1.00 = 100%
0.50 = 50%
0.20 = 20%
```

At 20% or below, guidance:

- Latches the mission-aborted state.
- Blocks further payload release.
- Cancels normal navigation.
- Commands landing in place if the mission has started.

Invalid battery percentages or battery data older than five seconds also trigger the battery safety response. Startup requires a valid battery reading.

PX4’s battery failsafe is separate from this mission logic.

### Emulator

The modified emulator battery:

- Starts at 100%.
- Drains only while armed.
- Reaches 20% after 90 cumulative armed seconds.
- Continues decreasing to a minimum of 0%.
- Pauses while disarmed.
- Resets when the emulator process restarts.

The timing starts when the emulator is armed, not when guidance starts.

Monitor battery readings with:

```bash
rostopic echo /uavasr/battery/percentage
```

For the real vehicle:

```bash
rostopic echo /mavros/battery/percentage
```

## Boundary Configuration

The stored guidance limits are:

```xml
<param name="x_min" value="-4.0" type="double" />
<param name="x_max" value="4.0" type="double" />
<param name="y_min" value="-3.2" type="double" />
<param name="y_max" value="2.0" type="double" />
<param name="z_min" value="0.5" type="double" />
<param name="z_max" value="4.0" type="double" />
```

These stored limits span 8 × 5.2 m horizontally; the current survey rectangle is 8 × 5 m.

**Guidance boundary checks are currently disabled by default** using:

```python
self.boundary_checks_enabled = rospy.get_param(
    "~boundary_checks_enabled", False)
```

To enable them, configure this private parameter before guidance starts:

```xml
<param name="boundary_checks_enabled" value="true" type="bool" />
```

The parameter is read at startup. Changing it while guidance is running does not update the existing instance.

When enabled, the existing boundary logic can command landing in place following a boundary violation. Review that response and the configured limits before enabling it, particularly during payload approaches.

This setting does not change PX4’s own geofence or failsafe settings.

## Fake Target Test

Run the test while guidance is actively surveying. Targets are ignored after search completion or a mission abort.

For a simulation-only test, first ensure no physical payload controller is connected to the actuator topics.

If the real object estimator is also publishing empty observations, those messages can reset the confirmation count. Temporarily stop it for the isolated test:

```bash
rosnode kill /object_position_estimator
```

If its launch configuration automatically respawns it, it must instead be excluded from the test launch.

Publish a fake backpack:

```bash
rostopic pub -r 5 /objects/observations vision_msgs/Detection3DArray \
"{header: {frame_id: 'map'}, detections: [{results: [{id: 1, score: 0.99, pose: {pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}]}]}"
```

Press **Ctrl+C after approximately two seconds**.

For a person, change:

```yaml
id: 1
```

to:

```yaml
id: 0
```

Expected behaviour:

```text
Confirm target
→ approach target
→ descend to 0.8 m
→ wait five seconds
→ publish actuator command
→ climb
→ resume survey
```

Restart the object position estimator with its original launch command after the test.

## Payload Controller

The current controller uses BCM GPIO numbering:

| Actuator | GPIO |
|---|---|
| Servo A | GPIO12 |
| Servo B | GPIO13 |

A `True` message requests the open position. A `False` message requests the closed position.

The current open command applies a 1.5 ms pulse at 50 Hz for approximately 0.5 seconds, then stops the pulses. This pulse width must be calibrated for the actual servo and mechanism; it does not guarantee a particular angle.

Guidance does not automatically send a close command after deployment.

A log saying `Payload A command sent` or `Payload B command sent` confirms software publication—not physical release. The controller has no release-position feedback.

## Troubleshooting

| Symptom | Checks |
|---|---|
| `ModuleNotFoundError: yaml` | Use the system Python, source ROS, and verify `python3-yaml` is installed |
| Guidance waits for SPAR | Check the SPAR node and flight-action topic configuration |
| Guidance waits for Breadcrumb | Check `/breadcrumb/request_path` and the planner terminal |
| Position startup timeout | Check `pose_source`, `pose_topic`, publisher availability, timestamps, and topic rate |
| Detection publishes but nothing happens | Check mission state, deployment setting, class ID, confidence, pose freshness, and competing publishers |
| Payload command appears but servo does not move | Check controller logs, power, common ground, GPIO wiring, calibration, and mechanism |
| Immediate low-battery abort | Check the selected battery topic and its percentage value |
| Emulator lands around 90 seconds after arming | Expected low-battery test behaviour |
| Unexpected boundary abort | Check whether boundaries are enabled, the actual pose, configured limits, and coordinate frames |
| New edits have no effect | Restart the affected node; rebuild first if C++ source changed |

Useful read-only commands:

```bash
rosnode list
rosnode info /guidance
rosparam get /guidance

rostopic hz /mavros/local_position/pose
rostopic hz /uavasr/odom

rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/battery

rostopic info /objects/observations
rostopic info /actuator_control/actuator_a
rostopic info /actuator_control/actuator_b
```

Press **Ctrl+C** to stop continuous commands.

## Known Limitations

- Target approach and return movements currently use direct SPAR waypoint commands rather than Breadcrumb obstacle planning.
- Position freshness checks do not provide a complete response to localisation loss during flight.
- Flight-action waits do not have a general maximum mission-duration timeout.
- Payload command publication does not confirm physical deployment.
- Camera coverage is not guaranteed solely by the waypoint pattern.
- Guidance boundary enforcement is currently disabled by default.
- Emulator mode does not automatically isolate physical actuator outputs.

## Development and Testing

Back up source and launch files before modifying mission behaviour.

After C++ emulator changes:

```bash
cd ~/catkin_ws_G6
source /opt/ros/noetic/setup.bash

catkin_make --pkg uavasr_emulator -j2
source devel/setup.bash
```

Restart the emulator to load the rebuilt program.

Validate changes in this order:

1. Syntax and build checks.
2. Isolated logic tests.
3. Emulator tests with physical actuators disconnected.
4. Controlled payload bench tests.
5. Supervised vehicle testing.

Ground actuator and flight-interface checks should be performed with the aircraft disarmed and propellers removed.
