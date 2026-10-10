# Autonomous Mission Migration Map

This document tracks how the legacy monolithic mission is being replaced by
the Avenue 2 autonomous mission architecture.

## Legacy reference

- Git commit: `fa07895`
- Legacy file: `src/spar/spar_node/scripts/test_breadcrumb_camera`
- Preserved working copy: `scripts/unused/test_breadcrumb_camera`

The Git commit is the authoritative reference because its contents and line
numbers cannot change as development continues.

## Migration table

| New component | Legacy predecessor | Status | Notes |
|---|---|---|---|
| `scripts/autonomous_mission:main()` | Legacy script startup section | Implemented | Thin ROS entry point only |
| `MissionController` | `Guidance` class | Implemented | Authoritative mission coordinator |
| `MissionState` | `Guidance` Boolean mode flags | Implemented | Replaces overlapping flags with one explicit state |
| `LEGAL_TRANSITIONS` | Implicit callback and method sequencing | Implemented | Defines permitted mission flow centrally |
| `transition_to()` | Direct flag changes throughout `Guidance` | Implemented | Validates and logs every state change |
| `run()` | Legacy ROS spin/wait behaviour | Implemented | Runs preflight, enters `TAKEOFF`, starts the 20 Hz dispatcher and registers shutdown handling |
| Successful preflight transition | Legacy startup readiness checks before mission execution | Implemented | Moves the controller from `PREFLIGHT` to `TAKEOFF` only after the SPAR action server, Breadcrumb service and required vehicle state are available. No aircraft command is issued by the transition itself. |
| Breadcrumb planner | `/breadcrumb/request_path` service | Retained unchanged | Existing obstacle routing remains authoritative |
| Payload hardware node | `payload-v1.2.py` | Retained with acknowledgement output | Continues as a separate ROS process; the established servo commands are unchanged and each actuator now publishes its latched deployed state. |
| `MissionController.perform_preflight()` | SPAR readiness check, legacy lines 328–332 | Implemented | SPAR availability is handled as an explicit mission prerequisite |
| Dependency injection through `MissionController(flight, breadcrumb, vehicle, target_observations, target_evidence, operator, payload)` | ROS clients and subscriptions constructed inside legacy `Guidance.__init__()` | Implemented | Keeps mission decisions separate from flight, path-planning, telemetry, raw observations, evidence, optional operator communication and payload communication. |
| `BreadcrumbInterface.__init__()` | Breadcrumb setup, legacy lines 288–302 | Implemented | Preserves `/breadcrumb/request_path` behind a dedicated interface |
| `BreadcrumbInterface.wait_until_ready()` | Legacy indefinite Breadcrumb wait, lines 291–297 | Implemented | Makes planner availability an explicit preflight prerequisite |
| `VehicleState` pose handling | `start_pose_input()`, `accept_pose()`, `callback_pose()` and `callback_odom()`, legacy lines 1032–1082 | Implemented | Normalises real and emulator pose inputs without making mission decisions |
| `VehicleState` battery handling | `callback_battery()` and `battery_ready()`, legacy lines 917–936 | Implemented | Separates battery observation from abort policy |
| `VehicleState.pose_is_ready()` | `pose_ready()`, legacy lines 1052–1058 | Implemented | Preserves local-position freshness checks |
| `VehicleState.battery_is_ready()` | `battery_ready()`, legacy lines 929–936 | Implemented | Reports battery readiness without directly aborting |
| `VehicleState.wait_until_ready()` | Startup pose and battery waits, legacy lines 334–351 and 1040–1050 | Implemented | Combines telemetry prerequisites for `PREFLIGHT` |
| `BreadcrumbInterface.request_path()` | Breadcrumb request and path selection inside `check_waypoint_status()`, legacy lines 1617–1763 | Implemented | Preserves sparse-path preference with normal-path fallback |
| `Waypoint` | Four-value waypoint lists `[x, y, z, yaw]` used throughout `Guidance` | Implemented | Replaces positional lists with immutable named fields |
| Breadcrumb pose-to-waypoint conversion | Conversion loop in `check_waypoint_status()`, legacy lines 1791–1829 | Implemented | Preserves returned x/y/z values and zero-yaw behaviour |
| `FlightInterface.send_waypoint()` | `send_wp()`, legacy lines 1487–1555 | Implemented | Preserves the established non-blocking SPAR `MOTION_GOTO` goal fields and motion parameters |
| SPAR motion configuration | Legacy lines 213–238 | Implemented | Preserves velocity and convergence parameter names and defaults |
| `FlightInterface.goal_state()` | SPAR status polling in `check_waypoint_status()`, beginning at legacy line 1592 | Implemented | Translates ROS action states into mission-facing flight states |
| `FlightInterface.cancel_current_goal()` | Direct `spar_client.cancel_goal()` calls in abort and shutdown handling | Implemented | Keeps SPAR cancellation inside the flight boundary |
| `MissionController._update()` and state-specific handlers | Legacy 20 Hz timer and `check_waypoint_status()`, lines 398–405 and 1574 onward | Implemented | Preserves periodic mission supervision while replacing the legacy all-purpose polling callback with one handler per explicit mission state. This separates route following, target handling, deployment and recovery decisions instead of accumulating them in one callback. |
| Thread-safe state transitions | Legacy `safety_lock` and mission flags | Implemented | Centralises concurrent state access and records state-entry time. |
| `FlightInterface.send_takeoff()` | Standalone `scripts/takeoff`; integrated legacy mission used its first `MOTION_GOTO` waypoint as an implicit take-off | Implemented | Uses SPAR’s dedicated `MOTION_TAKEOFF` command while retaining the established mission velocity and convergence parameters. This intentionally separates vertical take-off from movement toward the first search waypoint. |
| `MissionController._handle_takeoff()` | Legacy first-waypoint startup sequence, lines 376–405, together with standalone `scripts/takeoff` | Implemented | Submits one dedicated SPAR take-off goal, supervises it without blocking ROS callbacks, enters `SEARCH` only after success and sends failures to `ABORT`. |
| `build_default_search_plan()` | Fixed `waypoints` list in legacy `main()` | Implemented | Preserves the established lawnmower geometry as high-level search destinations while leaving obstacle-aware routing to Breadcrumb. |
| `VehicleState.current_position()` | Legacy `current_location` updated by `callback_pose()` or `callback_odom()` | Implemented | Returns a copied map-frame position only while the underlying pose remains valid and fresh, providing Breadcrumb with a safe route starting point. |
| Search and route indices | Legacy `nextWaypointIndex`, `breadcrumbWPS`, `breadcrumbWPSnextIndex` and `breadcrumbMode` | Implemented | Separately records progress through the high-level lawnmower plan and the intermediate waypoints in the currently active Breadcrumb leg. An active route replaces the redundant `breadcrumbMode` Boolean. |
| `MissionController._request_current_search_route()` | Breadcrumb request construction inside legacy `check_waypoint_status()`, lines 1617–1763 | Implemented | Requests each search leg online using the fresh measured aircraft position and the current lawnmower destination. Failed requests are rate-limited instead of repeatedly calling the service at 20 Hz. |
| `/guidance/fullPath` and `/guidance/pathBreadcrumb` publishers | Legacy RViz/GCS `nav_msgs/Path` publishers and `display_path()` | Implemented | Preserves the established latched visualization topics. The first displays the complete high-level search plan; the second displays the current obstacle-aware search, diversion or centring route without participating in flight control. |
| `MissionController._waypoint_is_within_boundary()` | Legacy `check_waypoint()`, lines 905–917 | Implemented | Preserves x/y/z boundary validation for commanded waypoints. The check is enabled by default and remains explicitly visible in the standard tmux mission commands. |
| `MissionController._send_next_route_waypoint()` | Boundary, battery and `send_wp()` calls inside legacy route progression | Implemented | Submits one validated Breadcrumb waypoint at a time, pauses for stale pose data, and advances the route index only after SPAR accepts the command. |
| `MissionController._handle_search()` | Route-following portion of legacy `check_waypoint_status()`, lines 1574 onward | Implemented | Preserves online one-leg-at-a-time Breadcrumb planning and sequential SPAR waypoint execution while making search progression an explicit state-owned behaviour. |
| `LandingMarkerInterface` | Legacy `aruco_locations`, `callback_aruco_markers()` and terminal input thread | Implemented | Retains persistent positions from `/world_model/aruco_markers`, prints observed IDs and validates the numeric ArUco ID entered in the mission pane without blocking ROS callbacks. |
| `WAIT_FOR_LANDING_SELECTION` | Legacy `begin_manual_selection()` and `wait_for_landing_selection()` | Implemented | Holds the aircraft after normal mission completion while the mission pane requests a detected landing ArUco. Emergency `ABORT` does not wait for operator selection. |
| `APPROACH_LANDING` | Legacy `fly_landing_location()` | Implemented | Uses Breadcrumb to reach the selected marker at survey altitude, descends to the configured 0.8 m approach altitude, then permits the normal `LAND` state. |
| `FlightInterface.send_land()` | Standalone `scripts/land`, legacy `send_land_motion()` and abort landing path | Implemented | Centralises the established SPAR `MOTION_LAND` command and 0.2 m/s default descent speed. Landing remains available during low-battery handling. |
| `MissionController._handle_land()` | Legacy `send_land_motion()` | Implemented | Submits one non-blocking controlled landing command, supervises SPAR completion and enters `COMPLETE` only after success. |
| `MissionController._handle_abort()` | Legacy `abort_mission()` and `abort_landing_done()` | Implemented | Cancels the active goal, makes one landing-in-place attempt and remains in `ABORT` with an operator warning if controlled landing cannot be completed. |
| `MissionController._handle_complete()` | Legacy successful landing calling `rospy.signal_shutdown("complete")` | Implemented | Treats `COMPLETE` as a terminal state and requests ROS shutdown exactly once after successful landing. |
| `MissionController.shutdown()` | Legacy `shutdown()` callback | Implemented | Cancels the active SPAR goal when the node is interrupted before completion and records the final mission state. |
| `VehicleState.external_position_age()` | Legacy mission inferred navigation health only from its current pose input | Implemented | Separately tracks the direct OptiTrack feed on real hardware and emulator odometry during simulation, allowing a true 10-second source-loss policy without confusing OAK-D imagery with external flight positioning. |
| External-position preflight check | No distinct direct OptiTrack-source check in the legacy controller | Implemented | Requires a fresh direct OptiTrack observation before real take-off; emulator odometry provides the equivalent simulation signal. |
| `MissionController._runtime_safety_allows_update()` | Legacy live pose supervision and `abort_mission()` | Implemented | Allows brief source interruptions but enters `ABORT` after ten continuous seconds without external-position telemetry. OAK-D imagery is not part of this abort condition. |
| `MissionController._live_boundary_allows_update()` | Legacy `check_live_boundary()` | Implemented | Preserves continuous x/y/z boundary supervision, permits initial ascent through `z_min`, and excludes landing states so controlled descent remains possible. |
| `TargetObservationBuffer` | Legacy mission subscribed directly to `Detection3DArray` | Implemented | Validates and stores complete raw detector frames, including empty frames, without tracking, evidence accumulation or mission decisions. |
| `TargetEvidenceAccumulator` | Legacy response to individual `Detection3DArray` observations | Implemented | Accumulates bounded, decaying evidence in separate person and backpack spatial clusters while preventing the 20 Hz mission loop from recounting one camera frame. It is not a tracker and makes no mission transition. |
| Class-specific coverage weighting | No legacy equivalent | Implemented with neutral tables | Provides separate twenty-band person and backpack weight tables. Every weight remains `1.0` until verified Avenue 1 results justify changing it. |
| `MissionController._strongest_confirmed_candidate()` | Legacy immediate response to a qualifying observation | Implemented and connected | Requires a fresh candidate, minimum observation count, available class-specific payload and class-specific evidence threshold. Initial confirmation permits investigation only, not deployment. |
| Active-candidate identity | Legacy target flags and latest detection state | Implemented | Retains the selected spatial-cluster ID and class while retrieving current position, score, confidence, image centre and coverage from fresh evidence snapshots. |
| Obstacle-aware `DIVERT` handling | Legacy target waypoint behaviour embedded in `check_waypoint_status()` | Implemented and reachable | Plans and supervises a separate Breadcrumb route above the candidate at survey altitude. Search progress remains separate and target depth cannot directly command descent. |
| Image-based `CENTRE` handling | No equivalent explicit mission state | Implemented and reachable | Uses fresh normalised bounding-box centres, a bounded hold time and Breadcrumb-planned corrections toward the candidate map position. Stale imagery causes hovering and the overall operation is timeout-bounded. |
| `MissionController._handle_reassess()` | Legacy confirmation flags mixed into waypoint supervision | Implemented | Requires new, fresh, centred observations after arrival. Loss of centring returns to `CENTRE`; failure to confirm returns to search after a bounded timeout. |
| `OperatorInterface` and veto messages | No reliable legacy equivalent | Implemented | Distinguishes fresh images actually received by Ground Control from onboard publication and accepts only candidate-matched deploy/veto responses. |
| `scripts/gcs_operator_bridge` | No legacy equivalent | Implemented | Runs on Ground Control, acknowledges received compressed images and offers the optional timed terminal veto. It is not launched onboard. |
| Candidate suppression and blacklist | Legacy target flags | Implemented | Unsuccessful investigations receive a short retry delay. An operator veto blacklists that class and location for the rest of the mission without suppressing the other class. |
| `PayloadInterface` | Direct actuator publishers in legacy mission logic | Implemented | Preserves class `0 = person` to actuator A and class `1 = backpack` to actuator B, waits for the separate payload node, and supports command acknowledgement. With deployment disabled it simulates acknowledgement without publishing a GPIO command. |
| `MissionController._handle_deploy()` | Legacy target-triggered release sequence | Implemented | Descends vertically to the configured release height, commands and verifies the class-mapped payload, then climbs vertically back to survey height. |
| `MissionController._handle_return_to_search()` | Legacy breadcrumb/target mode flags | Implemented | Clears transient target handling and replans the interrupted high-level search leg from the measured aircraft position. |
| `launch/nav_mission.launch` | Legacy mission parameters spread across script defaults and launch files | Implemented | Starts `autonomous_mission` with explicit evidence, centring, veto, blacklist and deployment parameters while allowing the payload node to remain a separate process. |
| `run349` / `emu349` mission pane | Legacy `test_breadcrumb_camera` command | Implemented | Uses `nav_mission.launch`; real flight retains manual arming and a separately launched payload node, while the emulator-only preflight re-arms `/uavasr` before each repeat mission and normally disables physical deployment. Starting either launcher intentionally replaces the existing `cdrone` session; `attach349` reconnects without restarting it. |
| `TestTargetInterface` and `/mission/inject_test_target` | Legacy `fake_roi` topic shortcut | Implemented; runtime test pending | When explicitly enabled, accepts `person` or `backpack`, samples a random free `/grid` cell within 2 m of the aircraft and inside the established search footprint with obstacle clearance, and asks the controller to exercise diversion and class-mapped deployment without forging detector evidence. |
| Terminal `person` and `backpack` commands | No equivalent concise mission test | Implemented; runtime test pending | Shell helpers call the guarded injection service. Requests are rejected outside `SEARCH`, without fresh pose/safe battery, or after that class payload has been used. |


## Current responsibility boundaries

| Responsibility | Owning component |
|---|---|
| Create and manage the SPAR action client | `FlightInterface` |
| Check whether SPAR is available | `FlightInterface.wait_until_ready()` |
| Coordinate mission prerequisite checks | `MissionController.perform_preflight()` |
| Decide how a failed prerequisite affects the mission | `MissionController` |
| Validate and record mission-state changes | `MissionController.transition_to()` |
| Construct and connect mission components | `scripts/autonomous_mission:main()` |
| Calculate obstacle-avoiding paths | Breadcrumb `/breadcrumb/request_path` service |
| Operate physical payload servos | `payload-v1.2.py` |
| Command and observe class-mapped payload state | `PayloadInterface` |
| Decide when payload deployment is permitted | `MissionController` |
| Report fresh imagery received at Ground Control | `scripts/gcs_operator_bridge` |
| Exchange candidate-specific veto requests and responses | `OperatorInterface` |
| Discover and communicate with the Breadcrumb service | `BreadcrumbInterface` |
| Choose when and where a path is required | `MissionController` |
| Calculate the obstacle-avoiding route | Breadcrumb planner |
| Subscribe to and validate vehicle pose | `VehicleState` |
| Observe the direct OptiTrack feed independently of PX4 local position | `VehicleState` |
| Subscribe to and validate battery telemetry | `VehicleState` |
| Decide whether unsafe telemetry requires abort | `MissionController` |
| Validate and retain complete raw target frames | `TargetObservationBuffer` |
| Accumulate decaying class-separated spatial evidence | `TargetEvidenceAccumulator` |
| Decide whether evidence permits investigation | `MissionController` |
| Calculate diversion and centring paths around obstacles | Breadcrumb planner |
| Decide whether a fresh target box is stably centred | `MissionController` |
| Decide whether a vetoed candidate location remains excluded | `MissionController` |
| Validate explicit test-target commands and sample free occupancy-grid cells | `TestTargetInterface` |
| Authorise the test diversion and bypass visual confirmation | `MissionController` |

## Vision-to-navigation interface table

This table tracks changes made on the DepthAI side specifically to supply
navigation with complete per-frame observations. These changes do not alter
detector training, raw detector evaluation, tracking or mission confirmation.

| DepthAI-side component | Existing predecessor | Status | Purpose |
|---|---|---|---|
| `TargetObservation.msg` | Information split between `Detection2D` and `Detection3D` | Implemented and generated | Carries one raw detection's class, confidence, image box, image dimensions, map position and depth-quality measurements together |
| `TargetObservationArray.msg` | Separate `/yolo/detections` and `/objects/observations` array headers | Implemented and generated | Groups observations from one source frame under a shared timestamp, frame ID and sequence number |
| Rich output from `object_position_estimator.py` | Existing `Detection3DArray` publication | Implemented | Publishes `TargetObservationArray` alongside the unchanged compatibility output, preserving raw class/confidence while adding image geometry and depth quality. |
| OAK sequence propagation in `dai_publisher.py` | Headers previously shared timestamps but retained the default sequence | Implemented | Copies the DepthAI RGB sequence number into compressed RGB, raw RGB, CameraInfo, aligned depth, YOLO detections and therefore the derived target-observation array. |
| Timestamped pose history in `object_position_estimator.py` | Latest-pose-only map transform | Implemented | Retains a bounded pose history and selects the pose nearest each source image timestamp, accounting for OAK inference latency without tracking or smoothing detector output. |

The table above records the DepthAI publisher, object-position estimator,
message-generation and mission-launch changes separately from navigation
policy.

## Architecture and behaviour diagrams

These diagrams record the implemented structure and intended behaviour. The
migration table remains the authority for implementation status, while the
runtime-validation section records what has actually been exercised.

### Component responsibilities

```text
scripts/autonomous_mission
        |
        | constructs and connects
        v
MissionController
   |          |             |             |                |
   v          v             v             v                v
Flight     Breadcrumb    VehicleState   Operator        Payload
Interface  Interface                    Interface       Interface
   |          |             |             |                |
   v          v             v             v                v
SPAR       Breadcrumb    MAVROS /       GCS bridge      separate
action     planner       emulator       (optional)      payload node
server
```

`MissionController` owns policy. The interface classes own communication with
their respective ROS services, actions and topics.

### Vision-to-navigation observation flow

```text
OAK-D RGB frame
      |
      v
Raw YOLO detections
class + confidence + image bounding box
      |
      +-------------------+
                          |
Aligned OAK-D depth ------+----> ObjectPositionEstimator
                          |              |
PX4 local pose -----------+              v
                               TargetObservationArray
                               one untracked array per frame
                                          |
                                          v
                                target-evidence component
                                          |
                                          v
                                  MissionController
```

```text
DepthAI / perception supplies facts:
    what class was detected?
    how confident was YOLO?
    where was its image box?
    where is it estimated in the map?
    how consistent was the depth?

Avenue 2 navigation supplies decisions:
    does evidence persist?
    does it cluster spatially?
    should search divert?
    is deployment permitted?
```

No tracker or temporal smoothing is inserted into this interface. Avenue 1
continues to evaluate raw detector output, while Avenue 2 consumes the same
raw observations and applies explicit mission evidence logic afterward.

### Perception and vehicle-pose timestamp alignment

```text
Vehicle pose stream at approximately 50 Hz
        |
        v
bounded two-second pose history
        |
        | select pose nearest source-image timestamp
        v
YOLO detection + exactly synchronized aligned depth
        |
        +-- nearest pose within 0.20 s --> map-frame target observation
        +-- no suitable pose ----------> publish an empty observation frame
```

The OAK result may arrive roughly half a second after image capture because
inference and transport take time. Selecting from timestamped pose history
uses the aircraft pose associated with capture time rather than comparing the
old image only against the newest pose. This aligns sensor data without
tracking, smoothing or modifying YOLO output.

### Raw target-observation buffering

```text
TargetObservationArray
one message per source RGB frame
          |
          v
validate every observation
class, confidence, box, map position and depth quality
          |
          v
store one immutable ObservationFrame
timestamp + sequence + receipt time + validated observations
          |
          v
MissionController reads the latest complete frame
```

An empty observation array is still stored as a complete frame. It records
that the detector processed an image without reporting a supported target;
it is not treated as missing camera communication.

### Target-evidence accumulation

```text
Latest ObservationFrame
          |
          v
same source frame already processed?
     +----+----+
     |         |
    yes        no
     |         |
  ignore       v
         decay every existing
         candidate score
               |
               v
       for each raw detection
               |
               v
   closest unused candidate of the same class
   within the horizontal clustering radius?
          +----+----+
          |         |
         no        yes
          |         |
      create a      update the candidate's
      candidate     averaged map position
          |         |
          +----+----+
               |
               v
   minimum contribution interval elapsed?
          +----+----+
          |         |
         no        yes
          |         |
      record the    add one bounded
      observation   evidence contribution
```

Person and backpack observations use separate cluster collections and
separate coverage-weight tables. Nearby detections of different classes can
therefore never reinforce the same candidate.

```text
one evidence contribution
          =
YOLO confidence
          x
class-specific coverage-band weight
          |
          v
limited by maximum_observation_contribution
```

Coverage is divided into twenty 5-percentage-point bands. The initial
tables remain neutral, so coverage does not yet raise or lower a score.
Verified Avenue 1 precision and recall results may later populate the person
and backpack tables independently. This is Avenue 2 evidence handling; it
does not alter or smooth the raw detector output evaluated by Avenue 1.

The minimum contribution interval prevents adjacent correlated camera frames
from being counted as fully independent proof. `advance_time()` continues
decaying scores when no new detector frame arrives, so a camera interruption
cannot leave old target evidence frozen indefinitely.

### Target-memory lifetime and reset status

```text
New detector frames
        |
        v
Latest raw frame is replaced continuously
        |
        v
Person/backpack evidence clusters accumulate separately
        |
        +-- unsupported for 1 s --> no longer eligible for diversion
        |
        +-- unsupported for 2 s --> score halves
        |
        +-- mission node restart --> clusters and latest frame are recreated empty
```

Evidence scores decay, but candidate records remain in memory for the lifetime
of the mission node. There is currently no runtime command that clears target
evidence. Restarting `autonomous_mission` clears it, but is only suitable while
safely on the ground because shutdown cancels the current goal and the new
mission begins startup and take-off again.

A future `clear_detections` command is proposed but not implemented. It should
be accepted only in `SEARCH` and clear the raw target buffer, accumulated
person/backpack evidence and temporary retry suppression while preserving
search progress, ArUco locations, mission-long veto blacklists and payload-use
state.

### Initial confirmation, diversion and centring

The target-handling path is connected. Passing initial confirmation interrupts
the current SPAR search goal but preserves the high-level search destination,
allowing the interrupted leg to be replanned afterward.

```text
SEARCH
  |
  | strongest fresh candidate satisfies:
  | minimum observations + class evidence threshold
  v
select active candidate by spatial-cluster ID and class
  |
  | cancel active search goal and preserve search-plan index
  v
DIVERT
  |
  | Breadcrumb route to candidate X/Y at survey altitude
  | sequential SPAR waypoint supervision
  v
CENTRE
  |
  +-- image observation stale ----------> hover and wait
  |
  +-- fresh but outside tolerance ------> Breadcrumb correction
  |
  +-- fresh and within tolerance
          |
          | remain centred for configured hold time
          v
       REASSESS
          |
          +-- confirmation lost --> RETURN_TO_SEARCH
          +-- confirmation passes --> optional veto/deployment path
```

```text
Candidate map Z / measured depth
            X
            |
            | does not directly command aircraft height
            v
Diversion and centring destination height = configured survey altitude
```

The centring timeout bounds both temporary detection loss and unsuccessful
position corrections. It returns control toward search rather than allowing
an indefinite hover. Breadcrumb remains authoritative for obstacle avoidance
during both the initial diversion and subsequent lateral corrections.

### Startup and preflight

```text
Start autonomous_mission
        |
        v
Construct flight, Breadcrumb, vehicle, observation, evidence,
operator and payload interfaces
        |
        v
Construct MissionController in PREFLIGHT
        |
        v
Check SPAR + Breadcrumb + required vehicle state
        |
        +-- failure --> ABORT (no aircraft command)
        |
        +-- success --> TAKEOFF
                         |
                         v
                   Start 20 Hz dispatcher
```

Changing state does not itself command the aircraft. Motion is issued only by
the handler responsible for that state.

### Launcher behaviour

```text
run349
  -> intentionally replace any existing cdrone session
  -> real MAVROS/Pixhawk, real payload process, real pose topics

emu349
  -> intentionally replace any existing cdrone session
  -> UAVASR emulator, emulator pose/battery, simulated payload acknowledgement

attach349
  -> attach to the existing cdrone session without restarting any node
```

The emergency pane prints the shutdown command but does not preload it at the
shell prompt. The tmux window follows the largest attached client so a small
diagnostic terminal does not unnecessarily constrain the main display.

### Periodic state dispatch

```text
ROS timer calls MissionController._update() at 20 Hz
        |
        v
Read the one active MissionState under the state lock
        |
        v
Look up its dedicated handler
        |
        +-- SEARCH ----------> _handle_search()
        +-- DIVERT ----------> _handle_divert()
        +-- CENTRE ----------> _handle_centre()
        +-- REASSESS --------> _handle_reassess()
        +-- other state -----> corresponding handler
```

This preserves the legacy periodic-supervision pattern while replacing the
large `check_waypoint_status()` callback with behaviour owned by explicit
states.

### Take-off supervision

```text
Enter TAKEOFF
      |
      v
Has the command already been sent?
  +-- no --> recheck fresh pose and safe battery
  |              |
  |              v
  |          send one MOTION_TAKEOFF goal
  |
  +-- yes --> inspect SPAR goal state
                 +-- pending/active --> remain in TAKEOFF
                 +-- succeeded ------> enter SEARCH
                 +-- failed/cancelled -> enter ABORT
```

The dedicated take-off command intentionally replaces the legacy integrated
mission's use of its first `MOTION_GOTO` waypoint as an implicit take-off.
The standalone legacy `scripts/takeoff` provides the established SPAR command
contract.

### Search-plan and route indices

```text
High-level lawnmower plan
    destination 0
    destination 1  <--- _search_waypoint_index
    destination 2
          |
          | Breadcrumb expands this one leg
          v
Active obstacle-aware route
    intermediate waypoint 0
    intermediate waypoint 1  <--- _route_waypoint_index
    intermediate waypoint 2
```

The high-level index records coverage of the search area. The route index
records progress around obstacles on the current leg. A non-empty active route
replaces the legacy `breadcrumbMode` Boolean.

### Search-route planning and retry

```text
No active Breadcrumb route
        |
        v
Has the retry delay elapsed?
    +-- no --> remain holding
    +-- yes
          |
          v
Obtain latest fresh measured position
        |
        v
Select current high-level lawnmower destination
        |
        v
Request Breadcrumb route
    +-- unavailable/no solution --> hold and retry after 1 second
    +-- valid route -------------> store route at index 0
```

Using the measured position improves on the legacy assumption that the
aircraft starts every leg exactly at the previous planned destination. The
retry interval also avoids calling the synchronous service on every 20 Hz
update when no route is available.

### Obstacle-aware movement

```text
MissionController chooses a destination
        |
        v
BreadcrumbInterface requests an obstacle-avoiding path
        |
        v
Breadcrumb planner returns sparse path (preferred) or normal path
        |
        v
Convert path poses into immutable Waypoints
        |
        v
MissionController selects the next Waypoint
        |
        v
FlightInterface sends a non-blocking SPAR goal
        |
        v
MissionController observes goal state and advances the route
```

The established Breadcrumb obstacle-routing algorithm remains unchanged.

### Waypoint dispatch safety gates

```text
Select next intermediate Breadcrumb waypoint
        |
        v
Fresh vehicle pose?
    +-- no --> hold; telemetry-loss policy remains authoritative
    +-- yes
          |
          v
Battery safe and fresh?
    +-- no --> ABORT
    +-- yes
          |
          v
Waypoint inside boundary when checks are enabled?
    +-- no --> ABORT
    +-- yes
          |
          v
SPAR accepts command?
    +-- no --> ABORT
    +-- yes --> advance the route index
```

The route index advances when SPAR accepts a command. Arrival is handled
separately by the state handler using the reported SPAR goal state.
An out-of-bound Breadcrumb waypoint is rejected immediately before submission
to SPAR. The controller enters `ABORT`; it does not ask Breadcrumb to replan.
On the following 20 Hz mission update, the abort handler cancels the active
goal and attempts controlled landing. Replanning the same request would not
currently add a boundary constraint to Breadcrumb and could reproduce the
same route.

### Complete search-leg cycle

```text
Enter SEARCH after take-off
        |
        v
Select current high-level lawnmower destination
        |
        v
Already within the established position tolerance?
    +-- yes --> advance to the next high-level destination
    +-- no
          |
          v
Request one online Breadcrumb route
        |
        v
Send one intermediate waypoint
        |
        v
Inspect SPAR goal state
    +-- pending/active --> wait
    +-- failed/cancelled -> ABORT
    +-- succeeded
          |
          v
More intermediate waypoints in this leg?
    +-- yes --> send the next waypoint
    +-- no  --> mark the high-level destination complete
                 |
                 v
             plan the next leg
                 |
                 v
All high-level destinations complete
        |
        v
WAIT_FOR_LANDING_SELECTION -> APPROACH_LANDING -> LAND
```

This retains online, one-leg-at-a-time Breadcrumb planning and sequential SPAR
execution. Target evidence interrupts `SEARCH` only through the explicit
`SEARCH` to `DIVERT` transition.

### Common runtime safety gates

```text
20 Hz mission update
        |
        v
Direct OptiTrack feed absent continuously for 10 seconds?
    +-- yes --> ABORT
    +-- no
          |
          v
Boundary checking enabled and measured position outside the volume?
    +-- yes --> ABORT
    +-- no
          |
          v
Run the handler for the active mission state
```

The local-position freshness limit and the OptiTrack-loss timeout serve
different purposes. A position older than one second cannot be used to plan or
submit new movement. An OptiTrack outage must persist for ten continuous
seconds before it forces abort landing.

### Live altitude-boundary behaviour

```text
TAKEOFF below z_min
        -> permit ascent through the lower boundary

Aircraft reaches or exceeds z_min once
        -> remember that normal flight altitude has been reached

Normal flight later drops below z_min
        -> ABORT

LAND or ABORT descent
        -> bypass live boundary check so landing remains possible
```

Waypoint validation prevents the controller from deliberately commanding an
outside destination. Live-boundary supervision independently detects the
aircraft actually crossing the configured volume.

### Normal and abort landing

```text
Normal mission completes
        |
        v
WAIT_FOR_LANDING_SELECTION
        |
        +-- unknown/non-numeric ID --> ask again while hovering
        |
        +-- observed ArUco selected
                    |
                    v
          Breadcrumb route at survey altitude
                    |
                    v
          descend to 0.8 m above marker
                    |
                    v
LAND sends one MOTION_LAND goal
    +-- succeeded --> COMPLETE --> ROS shutdown
    +-- failed ----> ABORT
                         |
                         v
                 cancel current SPAR goal
                         |
                         v
                 attempt landing in place once
                    +-- succeeded --> COMPLETE --> ROS shutdown
                    +-- failed ----> remain in ABORT
                                      request operator intervention
```

Landing is deliberately not blocked by low-battery telemetry. Repeated abort
landing commands are also prohibited because resubmission could continually
replace an action already attempting to descend.

### Overall autonomous mission flow

```text
PREFLIGHT -> TAKEOFF -> SEARCH
                         |
              candidate confirmed
                         v
                       DIVERT -> CENTRE -> REASSESS
                                              |
                   +--------------------------+-----------------------+
                   |                          |                       |
            confirmation lost          confirmation passes     serious fault
                   |                          |                       |
                   v                          v                       v
           RETURN_TO_SEARCH          optional veto decision          ABORT
                   |                          |
                   +--------> SEARCH <--------+---- veto
                                              |
                                      deploy or timeout
                                              v
                                           DEPLOY
                                              |
                           +------------------+------------------+
                           |                                     |
                   more search required                    mission finished
                           v                                     v
                   RETURN_TO_SEARCH             WAIT_FOR_LANDING_SELECTION
                                                               |
                                                               v
                                                       APPROACH_LANDING
                                                               |
                                                               v
                                                        LAND -> COMPLETE
```

### Explicit test-target injection

```text
Launch with enable_test_target_injection:=true
        |
        v
Type person or backpack in any sourced RPi terminal
        |
        v
Mission is in SEARCH and that class payload is available?
    +-- no --> reject command without changing flight
    +-- yes
          |
          v
Select random free /grid cell inside lawnmower footprint,
within 2 m of the current aircraft position and with configured obstacle clearance
          |
          v
Cancel current search leg and use Breadcrumb to divert
          |
          v
Reach injected location at survey altitude
          |
          v
Bypass camera centring, reassessment and optional GCS veto
because the explicit test command is the authorisation
          |
          v
Normal DEPLOY sequence
descend -> class-mapped payload -> acknowledge -> ascend
          |
          v
Return to interrupted search, or select landing ArUco if both payloads used
```

This path tests navigation and deployment integration, not detector quality or
normal autonomous confirmation. The launch-file default is disabled, while
the standard `run349` and `emu349` mission commands explicitly enable it so
the one-word helpers are available in the project test environments. On real
flight, `person` or `backpack` can cause a physical release when `deploy:=true`.

### Search footprint, occupancy map and optional safety boundary

```text
High-level search footprint
x = -3.5 to 3.5 m, y = -2.5 to 1.5 m
        |
        v
Breadcrumb /grid occupancy map
80 x 80 cells at 0.1 m resolution, centred on map origin
occupied border and virtual-obstacle cells
        |
        v
Obstacle-aware routes between mission destinations

Separate mission safety boundary
x = -4.0 to 4.0 m, y = -3.2 to 2.0 m, z = 0.5 to 4.0 m
        |
        +-- commanded-waypoint check
        +-- continuously measured-position check
```

The earlier emulator and camera tests ran with boundary checks disabled, as
confirmed by the mission warning at startup. Boundary checking is now enabled
by default through `boundary_checks_enabled:=true`, and both `run349` and
`emu349` include that argument explicitly in the mission pane. Breadcrumb
obstacle avoidance and the fixed lawnmower geometry remain separate systems.

### Final confirmation and optional operator veto

```text
Autonomous target confirmation passes
        |
        v
Aircraft centres above and reassesses target
        |
        v
Autonomous confirmation and safety gates still pass?
        +-- no --> abort candidate and reacquire
        |
        +-- yes
             |
             v
Is GCS receiving fresh imagery?
        +-- no --> deploy autonomously
        |
        +-- yes --> offer timed operator veto
                         +-- deploy/yes -----> deploy
                         +-- no response ----> deploy autonomously
                         +-- veto/no --------> do not deploy
                                               blacklist candidate location
                                               resume search for that class
```

Ground Control imagery and affirmative approval are not prerequisites for
autonomous deployment. Fresh imagery only determines whether offering the
operator a veto is useful.

### Deployment and return-to-search

```text
Enter DEPLOY with a confirmed class-specific candidate
        |
        v
Hold current X/Y and descend to configured deployment altitude
        |
        v
Command mapped payload
person (class 0) -> actuator A
backpack (class 1) -> actuator B
        |
        v
Payload acknowledgement received before timeout?
    +-- no --> ABORT
    +-- yes
          |
          v
Hold deployment X/Y and climb to survey altitude
          |
          +-- both payloads used --> WAIT_FOR_LANDING_SELECTION
                                      |
                                      v
                              APPROACH_LANDING -> LAND
          +-- one payload remains --> RETURN_TO_SEARCH
                                      |
                                      v
                         clear target-only state and replan
                         interrupted search leg from current pose
```

With `deploy:=false`, `PayloadInterface` simulates the acknowledgement without
publishing a servo command. This permits the same state sequence to be tested
in the emulator without GPIO hardware.

### Onboard and Ground Control boundary

```text
ONBOARD                                      GROUND CONTROL

compressed camera image -------------------> image receiver / display path
                                                  |
                                                  v
OperatorInterface <------------------------- exact image-header receipt
        |
        | candidate-specific request
        +-----------------------------------> gcs_operator_bridge
                                                  |
                                    optional deploy/veto response
        <-----------------------------------------+

No fresh receipt: onboard autonomy does not wait.
Fresh receipt but no response: onboard autonomy resumes after the timeout.
Veto: class-specific candidate location is blacklisted for this mission.
```

The GCS bridge must run on the computer receiving the imagery. Merely
publishing the camera image onboard does not count as Ground Control receipt.

### Telemetry-loss policy

```text
Real aircraft:
OptiTrack/VRPN -> /mavros/vision_pose/pose -> PX4 estimator
                                                   |
                                                   v
                                      /mavros/local_position/pose

Local pose older than 1 second
        -> treat the pose as unusable for navigation calculations

OptiTrack absent continuously for 10 seconds
        -> request mission abort / controlled termination

OAK-D or GCS imagery unavailable
        -> continue autonomous flight
        -> do not offer the optional operator veto
```

## Runtime-validation status

Verified on the project machine:

- Python syntax, Bash syntax, launch XML, executable permissions and Git diff
  whitespace checks pass.
- The emulator supplies SPAR, pose, odometry and battery data.
- `/yolo/detections` publishes at approximately 9.1 Hz.
- `/objects/target_observations` publishes at approximately 9.1 Hz with the
  expected `TargetObservationArray` type and source-image header.
- The timestamped pose-history estimator produced a non-empty map-frame target
  observation with depth-quality values. The observed person was classified
  by the candidate detector as backpack, which is a detector-quality result
  for Avenue 1 rather than a failure of the fusion transport.

Still to verify before real flight:

- `autonomous_mission` appearing as the target-observation subscriber;
- state transitions and Breadcrumb commands in emulation;
- restored `/guidance/fullPath` and `/guidance/pathBreadcrumb` visualization
  output after restarting the mission controller;
- end-of-mission ArUco prompting, Breadcrumb landing approach and final descent
  in emulation;
- simulated person and backpack deployment paths, retry suppression and veto
  blacklist behaviour;
- payload acknowledgement topics with physical hardware;
- Ground Control image receipts and the optional veto helper across the real
  network;
- provisional evidence, centring, timeout and deployment parameters using
  measured detector results and controlled flight tests.

## Migration rules

1. Preserve known-working ROS interfaces unless there is a measured reason to change them.
2. Record each replaced legacy function before removing its operational role.
3. Keep raw detector evaluation separate from mission confirmation logic.
4. Prove new behaviour in the emulator before switching the real-flight launcher.
5. Retain the legacy implementation through Git history for recovery.

## Telemetry-loss policy

- Local pose must be fresh enough for navigation calculations, using a
  one-second freshness limit.
- Loss of OptiTrack telemetry triggers mission abort only after a continuous
  ten-second timeout.
- Loss of OAK-D imagery does not trigger mission abort.
- OAK-D/GCS image availability only determines whether the optional operator
  veto can be offered.
- No additional mission-level response is implemented for total
  Pixhawk/MAVROS telemetry failure.
