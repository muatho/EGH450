# Take #2 Navigation Change Log

## Purpose

This file records changes made to `scripts/test_breadcrumb_camera` during the
Take #2 target-integration work. It distinguishes implemented behaviour from
proposals so the navigation owner can see exactly how the legacy mission was
altered.

## Reference point

- Legacy mission baseline: `scripts/test_breadcrumb_camera` at Git commit
  `fa07895`

## Scope boundary

The legacy mission remains responsible for:

- take-off and search progression;
- Breadcrumb route requests;
- SPAR waypoint execution;
- diversion, descent, climb and return movement;
- final ArUco landing selection.

Target-specific additions may determine:

- whether recent detections confirm a target;
- the stable target location supplied to navigation;
- whether the target remains visible and centred;
- whether deployment is permitted from the perception side.

Changes to general flight behaviour, SPAR, Breadcrumb or payload ownership must
be recorded and agreed separately.

## Implemented Take #2 changes

### Independent target world model

Initial target confirmation has been moved out of the navigation script into
`depthai_publisher/target_world_model.py`. The node subscribes to
`/objects/target_observations` and:

- retains the most recent N observation frames in a bounded `deque`;
- applies a confidence threshold for each target class;
- ignores observations without a valid map position;
- associates only same-class observations within the matching radius;
- counts at most one associated observation from each frame;
- confirms a cluster after support in at least M retained frames;
- returns the median confidence and median map position of the supporting
  observations;
- publishes every currently confirmed target on `/objects/confirmed_targets`;
- remembers spatially distinct confirmed targets for RViz on the latched
  `/world_model/target_markers` topic;
- accepts `/objects/locked_target` when the mission freezes a destination, so
  later observations cannot drag that marker across the map;
- prints one terminal message when a new spatial target is first confirmed.

Current configuration:

- M = 3 detected frames required for confirmation
- N = 5 recent frames retained
- minimum confidence = 0.60 for person and 0.40 for backpack
- spatial matching radius = 0.30 m
- minimum box coverage = 0.0% for each class, leaving the configurable size
  gate neutral

The coverage thresholds can be changed independently for person and backpack
without modifying code. Their current 0.0% values deliberately preserve all
otherwise qualifying detections; selecting a non-zero operational value is
model evaluation rather than unfinished integration. The mission subscribes
to `/objects/confirmed_targets` and retains the established legacy hand-off
into `callback_inspect_roi()`; Take #2 has not replaced Breadcrumb or SPAR
movement.

Current confirmations are deliberately distinct from remembered RViz markers.
A remembered marker is not sufficient to initiate flight towards an object;
the target must still satisfy the live 3-of-5 confirmation requirement.

The mission retains the preceding confirmed set so one continuous confirmation
does not start a new diversion every frame. During an active diversion, the
world-model node continues confirming and displaying other targets, but
navigation does not queue them. Once the diversion finishes, another target
may initiate a diversion only if it is still freshly confirmed.

### Target-observation interface

The target-world-model node and mission subscribe to
`/objects/target_observations`. Each observation contains class, confidence,
image box, image dimensions and an explicitly valid or invalid map position.
Empty arrays are still published when a frame contains no detections, allowing
missed frames to enter the M-of-N window. The mission uses the raw observations
only to update and finally reassess the target currently being investigated.

The object-position estimator continues publishing `/objects/observations` for
existing consumers. Its target output uses the aircraft pose nearest the source
image timestamp and estimates depth from the median of a fixed 7 x 7 patch at
the box centre.

The OAK frame sequence number is copied into the RGB, compressed-image, camera
information, aligned-depth and YOLO headers so corresponding sensor products
retain a shared frame identity.

### Deployed detector compatibility layer

The active deployment now uses the 416 x 416, 80-class COCO YOLOv5 blob and
matching JSON stored in `models/egh450_yolo5_coco_v2`. The DepthAI decoder
therefore remains configured for all 80 model outputs. `dai_publisher.py`
filters those outputs to COCO person class 0 and backpack class 24, then
publishes the stable application interface:

- class 0: person;
- class 1: backpack.

Every other COCO class is discarded before publication. This keeps the mission,
overlay and target-world-model code independent of the detector's original
class numbering.

RGB, stereo depth and inference are currently configured for 8 FPS. The
processed display topic is `/processed_image/compressed`. The overlay publishes
the newest ready frame and drops older presentation frames so delayed detector
results cannot make the displayed video run backwards or accumulate latency.
These presentation drops do not remove machine-readable detector messages.

### Minimum coverage gate for diversion

The target-world-model node calculates each box's coverage as:

`100 x box width x box height / (image width x image height)`

The box and image dimensions are both expressed in the 640 x 360 source-image
coordinate system. `dai_publisher.py` removes the 416 x 416 inference
letterbox padding before publishing the box, so the calculation does not mix
coordinate spaces.

Separate `person_min_coverage_percent` and
`backpack_min_coverage_percent` values are stored beside the confidence values
in `depthai_publisher/config/target_confidence.yaml`. Both the target world
model and navigation mission load this shared policy. The current value for
both is 0.0%. A detection below its class threshold remains available on the
raw detector and target-observation topics for display and logging, but cannot
support initial confirmation, active tracking, position updates or final
confirmation. Another qualifying detection in the same frame is still
processed. Coverage values are not written to a separate experimental log.

### Compact target reporting after deployment

Confirmed person and backpack reports now use the same compact class,
confidence and world-position format as the existing ArUco discovery report.

The vision pipeline continues publishing every valid target location after a
payload has been deployed. The mission no longer returns early when both
payloads are unavailable. Detections of a class whose payload has already been
deployed pass through a separate per-class 3-of-5 confirmation window. One
`IGNORED` event is then published for that continuous detection episode without
requesting another diversion. The event is rearmed after the target disappears,
allowing a later reacquisition to be reported once.

This changes terminal reporting and target filtering only. It does not change
the underlying object-observation messages or payload actuation.

### Target-event reporting and payload terminology

The mission uses the following operator-facing names while retaining the
existing actuator identifiers internally:

- person: EpiPen, actuator A;
- backpack: Tracker, actuator B.

The mission publishes compact target events on `/target/events` as
`std_msgs/String`. The existing `vision_overlay` node subscribes to this topic
and prints the messages in the imagery pane. Each event uses three readable
lines—event name, class/confidence and XYZ—with a blank line before the next
event.

The intended division between the two adjacent tmux panes is:

- imagery pane: target `DETECTED`, `CONFIRMED`, `REJECTED` and `IGNORED`
  events;
- navigation pane: diversion, holding, lowering, deployment, climbing and
  resumed-search actions.

`DETECTED`, `CONFIRMED`, `REJECTED` and `IGNORED` events are connected.

### Active-target evidence during diversion

Once initial M-of-N confirmation reserves a target, diversion runs in a worker
thread so `/objects/target_observations` callbacks can continue while the
legacy Breadcrumb/SPAR movement executes.

The active-target layer retains:

- the target class and initially acquired position;
- one matched observation or explicit miss for each of the latest five frames;
- median confidence, image-box centre and map position;
- the time of the latest accepted observation.

The position filter calculates the coordinate-wise median from detections that
remain inside that same five-frame window. A sixth frame automatically removes
the oldest frame, so a missed detection cannot leave an older position in the
median after its frame has left the window.

The ArUco path deliberately behaves differently. An ArUco ID uniquely
identifies a marker, so its stored position is replaced by the latest estimate
for that ID. YOLO does not intrinsically identify individual object instances;
its person and backpack locations therefore use recent temporal evidence and a
median to reduce position noise before navigation acts on them.

### Diversion destination policy

Initial 3-of-5 confirmation supplies the first target estimate, not a permanently
fixed destination. The mission requests an obstacle-safe Breadcrumb route to
that estimate while fresh detections continue updating a rolling five-sample
median.

Location refinement is limited to three seconds. During that period the
aircraft begins following the initial route and the rolling median continues
to update. At the end of the period, the latest fresh 3-of-5 median is copied
and frozen. Later observations cannot drag either the navigation destination
or its RViz marker.

The mission then requests one finite Breadcrumb route to the frozen location
and completes it using the ordinary 0.30 m SPAR waypoint radius. It verifies
that Breadcrumb kept the requested destination in the same free occupancy-grid
cell; it does not bypass an obstacle-induced endpoint displacement. The
three-second limit bounds only dynamic refinement, not the time required to
complete the fixed obstacle-safe route.

Once the position locks, the retained observations are cleared. Final
confirmation therefore uses only fresh observations obtained from above the
target. Failure to obtain fresh, centred and spatially consistent evidence
causes rejection and a return to the interrupted search rather than deployment.

A fresh observation supports the active target only when it has the same
class, meets that class's confidence and coverage thresholds, has a valid
finite map position and lies within `target_association_radius`, currently
0.30 m. If multiple detections qualify, the one nearest the active rolling
median is used. Medians are used so one noisy observation cannot dominate the
current estimate. `serviced_target_radius`, currently 1.00 m, prevents repeat
service near a location where deployment already succeeded.

Person and backpack mission-confidence thresholds are defined once in
`depthai_publisher/config/target_confidence.yaml`. The image-processing and
mission launches load that file into `/target_policy`, which both the target
world model and navigation mission read. This also keeps the values consistent
when the mission itself is started with `rosrun`. The raw DepthAI decoder
threshold remains a separate model deployment setting.

Direct legacy ROI commands and explicitly injected test targets retain their
fixed-target behaviour because neither represents a live detector track.

The detector-driven approach is:

```text
initial 3-of-5 confirmation
        -> request Breadcrumb route to P0
        -> fly and update the rolling median for 3 seconds
        -> freeze the latest fresh median
        -> request and complete one Breadcrumb route to the frozen location
             |-- endpoint displaced to another grid cell -> reject movement
             `-- route complete -> fresh final check, up to 3 seconds
```

### Final confirmation and return to search

`wait_for_final_target_confirmation()` has been added as the target-side check
to run once the legacy mission reaches the position above the target. It clears
pre-arrival evidence so the deployment decision uses only observations made
from the final viewing position.

Current requirements are:

- complete within a fixed three-second deadline that detections do not reset;
- each supporting detection passes its class confidence and coverage thresholds;
- target present in at least 3 of the latest 5 frames;
- newest supporting detection no more than 0.5 seconds old;
- median box-centre offset no more than 15% of the image width or height from
  the image centre, defining a central 30% acceptance region;
- maximum XY separation between any two retained map positions no greater than
  0.30 m.

The 0.15 centring tolerance is separate from the 0.30 m world-position
stability tolerance: one checks where the target appears in the image and the
other checks whether its estimated map location has settled.

The method returns the confirmed class, confidence and filtered position when
all requirements pass. `callback_inspect_roi()` now calls it after reaching the
position above the target.

If final confirmation fails, the mission returns to the interrupted search
route and then starts a ten-second suppression period for that location. It
then permits one later reacquisition using new
3-of-5 evidence. A successful check may return immediately rather than consume
all three seconds. If the later investigation also reaches and fails final
confirmation, the mission publishes `REJECTED`, removes and suppresses the
location's RViz marker and skips deployment for that location for the rest of
the mission. The imagery event and navigation warning append a short failure
reason such as `not tracked`, `not centred` or `global location inconsistent`.

If either final confirmation succeeds, the mission publishes `CONFIRMED`,
replaces the deployment waypoint X/Y with the filtered position, descends using
the ordinary SPAR waypoint radius, runs a five-second terminal countdown and
requests deployment. The deployment altitude remains the legacy 0.8 m by
current decision. The 10 cm customer requirement is assessed from measured
camera-relative localisation data rather than enforced as an unsupported
global-navigation gate.

The countdown now replaces the legacy silent five-second pre-deployment wait;
it does not add a second delay. `deploy_payload()` returns success only after
publishing the selected actuator command. The confirmed location is recorded
as serviced only on that success. The climb waypoint uses the same confirmed
X/Y as the deployment waypoint so the initial climb is vertical.

During the same five-second countdown, an operator may press `R` in the
interactive mission pane to reject deployment. The countdown updates one
terminal line rather than printing five separate lines. Operator rejection:

- prevents the actuator command;
- immediately blacklists that class/location for the mission;
- publishes `/objects/rejected_target`;
- removes and suppresses the corresponding persistent RViz marker;
- climbs and returns to the interrupted search route.

If the mission has no interactive terminal, it warns that manual rejection is
unavailable and the autonomous countdown continues.

When `~deploy:=false`, the mission does not wait for the payload controller,
does not reset either servo and does not publish deployment commands. Detection
and world-model reporting remain active, but targets cannot interrupt the
search. When deployment is enabled, the existing startup close commands and
class-to-actuator mapping remain in use.

A direct legacy `~roi` request has no person/backpack class and therefore
cannot choose an EpiPen or Tracker. It now flies to the requested above-target
position and returns to the interrupted route without target reassessment or
payload deployment.

### Explicit test-target injection

Two `std_srvs/Trigger` services provide deliberate end-to-end test inputs:

- `/mission/inject_person` for an EpiPen target;
- `/mission/inject_backpack` for a Tracker target.

The shell commands `person` and `backpack` call these services. The mission
chooses a random free occupancy-grid cell inside the search footprint, no more
than 2 m from the current aircraft position and with 0.3 m obstacle clearance.
Unknown and occupied cells are rejected.

An injected target uses the existing target diversion, Breadcrumb movement,
deployment, climb and return workflow. It bypasses camera-based final
confirmation because the terminal command is the explicit test instruction;
real detections still require the normal final reassessment. Injection does
not fabricate detector observations or enter the live 3-of-5 confirmation
output.

Injected locations are published on `/objects/injected_test_target`. The
independent target world model remembers them on the existing RViz marker
topic. Their terminal discovery line uses `TEST TARGET`, and their RViz label
is prefixed with `TEST`.

The `~deploy` and `~enable_test_target_injection` parameters both default to
`true`. The editable `emu349` mission command explicitly sets both to `false`,
so it is valid as written and starts without either behaviour. Deleting both
overrides enables the complete injected diversion and deployment test.
`run349` leaves normal deployment enabled but explicitly disables test-target
injection. On every direct `rosrun`, an omitted runtime switch is restored to
its script default before the mission reads it, so deleting an override works
even while the same ROS master remains active. Launch-file starts instead use
fresh private namespaces through `clear_params="true"`.

### Mission-start world-model reset

Starting the mission calls `/world_model/clear_aruco` and
`/world_model/clear_targets`. These services clear retained evidence and send
RViz `DELETEALL` markers before the new mission begins. The latched
`/guidance/cameraCoverage` and `/guidance/droneTrail` markers are also
explicitly deleted before their new mission histories are accumulated. A
service that is unavailable after two seconds produces a warning but does not
block mission startup. Live detections may repopulate the maps immediately
after the reset.

The camera-coverage trail reads the OAK-D intrinsics from
`/depthai_node/camera/camera_info`. Each rectangular ground footprint is
calculated from the image dimensions, focal lengths and current world-frame
height instead of using the previous fixed 1.4 m by 2.3 m rectangle. The
calculation assumes a downward-facing, approximately level camera and a ground
plane at world `z = 0`; it does not model aircraft roll or pitch.

The drone snail trail is published as a blue `LINE_STRIP` marker on
`/guidance/droneTrail`, with a new point after each 0.20 m of three-dimensional
movement. It replaces RViz's client-owned Odometry history when mission-start
reset behaviour is required.

### RViz target and landmark presentation

The independent world-model topics do not depend on the mission node or on an
RViz display being configured:

- `/world_model/aruco_markers`: red ArUco marker plus a white label containing
  marker ID and XYZ;
- `/world_model/target_markers`: blue person cube or yellow backpack cube plus
  a white class-and-XYZ label. Injected targets are prefixed with `TEST`.

Both topics are latched. RViz is a subscriber only; absence or failure of RViz
does not stop detection, mapping or navigation.

### GCS voice-announcement integration status

The target and mission voice nodes are GCS presentation components and are not
started by `run349`, `emu349` or the onboard mission launch files. The GCS
`gui450` function owns their launch through a combined wrapper and creates the
`gcs450` tmux session with three panes:

- both voice-announcement nodes;
- RQT;
- the existing `screenrecord` command presented for optional manual start.

The announcer launch automatically uses `/uavasr/state` when the emulator state
topic exists and otherwise uses `/mavros/state`. The GCS launch and speech path
have been exercised successfully. The announcer core unit tests also pass.

## Current integration limitations

The target flow is connected. User-observed emulator runs have exercised
initial confirmation, final confirmation, rejection/return-to-search and
backpack deployment. The saved scripts pass Python compilation and whitespace
validation, and the complete Catkin workspace has built successfully. The
duplicate delay, false serviced-target record, diagonal-climb coordinates and
direct `~roi` exception found during code review have been corrected.

Additional limitations remain:

- targets confirmed during another diversion are reported and displayed but
  are not placed in a navigation queue;
- a remembered RViz location does not itself cause a later diversion;
- minimum coverage thresholds currently remain at the deliberately neutral
  0.0% setting;
- some older navigation launch parameters remain and must be reconciled.

These are recorded limitations, not approved final behaviour.

## Agreed implementation order

1. Completed in user-observed emulator testing: initial confirmation, final
   confirmation, rejection/return and backpack deployment paths.
2. Completed: replace the repeated deployed-payload notice with one `IGNORED`
   event per reacquisition.
3. Completed; runtime verification pending: refine the rolling target median
   for three seconds, freeze it and complete one finite Breadcrumb route using
   the ordinary 0.30 m waypoint radius.
4. Completed: reintroduce independent person/backpack target markers and
   one-time terminal reporting for newly confirmed spatial targets.
5. Completed: reintroduce explicitly gated person/backpack test injection.
6. Completed; runtime verification pending: allow one three-second final check,
   start a ten-second cooldown when that check fails, return to search while the
   timer runs, permit one later reacquisition and blacklist only after that
   retry also fails final confirmation.
7. Completed: connect independent per-class minimum-coverage parameters and
   restore the final image-centre tolerance to 0.15. The current coverage
   values remain deliberately neutral at 0.0%.
8. Completed: remove the persistent RViz marker after either manual rejection
   or automatic failed-retry blacklisting, and visibly prefix injected target
   labels with `TEST`.
9. Completed and exercised: provide the timed operator veto through `R` during
   the five-second deployment countdown; no input continues autonomously.
10. Remaining: reorganise and label the mission script by subsystem
    responsibility without changing runtime behaviour.

Temperature sensing and additional preflight tooling are outside the remaining
Take #2 implementation scope.

## Legacy-to-Take #2 mapping

| Take #2 addition or replacement | Legacy location | Status | Reason |
| --- | --- | --- | --- |
| Independent `TargetWorldModel` node | Candidate counting inside `callback_object_observations()` | Implemented and connected | Keep bounded M-of-N confirmation and persistent perception memory outside navigation while preserving the existing diversion hand-off |
| `TargetObservationArray` subscription | Legacy `Detection3DArray` subscription | Implemented | Preserve image evidence and map validity together without changing the raw detector output |
| `/objects/confirmed_targets` hand-off | `callback_inspect_roi()` | Implemented | Let perception decide which locations are freshly confirmed while leaving Breadcrumb/SPAR diversion movement under the recognisable legacy mission |
| `/world_model/target_markers` | No direct legacy equivalent | Implemented | Display persistent, spatially distinct person and backpack locations without making remembered markers navigation commands |
| `/objects/locked_target` marker freeze | No direct legacy equivalent | Implemented; runtime verification pending | Keep the displayed location aligned with the mission's frozen destination instead of continuing to move it with later observations |
| Per-class minimum box coverage | No legacy equivalent | Implemented and exercised; thresholds currently 0.0% | Prevent undersized detections from authorising diversion without removing them from raw display/logging |
| Shared mission-confidence configuration | Duplicated 0.60 values in perception and mission code | Implemented | Keep initial diversion and final reassessment thresholds class-specific but editable from one YAML file |
| `/target/events` publisher and imagery-pane subscriber | No direct legacy equivalent | `DETECTED`, `CONFIRMED`, `REJECTED` and `IGNORED` connected | Separate perception decisions from navigation-action messages |
| Active-target worker and rolling evidence | Legacy synchronous diversion with no observation updates | Implemented; revised five-frame buffer requires runtime verification | Keep perception callbacks alive during diversion and calculate medians only from detections still inside the rolling frame window |
| Bounded rolling-target approach | Legacy fixed target waypoint | Implemented; runtime verification pending | Refine the target median for three seconds, freeze the destination and marker, then complete one finite obstacle-safe Breadcrumb route without allowing later observations to drag it |
| `wait_for_final_target_confirmation()` | No final perception reassessment | Implemented; revised retry policy requires runtime verification | Require fresh, persistent, centred and spatially stable evidence before deployment |
| Rejection return to search | Legacy target flow always continued towards deployment | Implemented; exercised in emulator | Skip deployment when final evidence fails and resume the interrupted route |
| Obstacle-safe diversion rejoin | Legacy sent one direct GOTO to the saved interruption position using the ordinary 0.30 m radius | Implemented; runtime verification pending | Plan the return through Breadcrumb, reach the saved interruption position within 0.10 m, then restore and resume the remaining lawnmower route |
| Target-evidence cooldown and retry | No direct legacy equivalent | Implemented; runtime verification pending | Start the ten-second cooldown when refinement evidence or the final check first fails, return to search while it runs, allow one later reacquisition and blacklist only if that investigation also fails |
| Five-second deployment countdown and `R` rejection | Legacy silent five-second wait in `deploy_payload()` | Implemented and exercised | Make the existing stabilisation delay visible and provide a local final veto without extending it |
| `/objects/rejected_target` marker removal | No direct legacy equivalent | Implemented for manual and automatic blacklisting | Keep the marker during the retry cooldown, then hide and suppress a location rejected by the operator or blacklisted after the failed retry |
| Boolean deployment result and serviced-location update | Legacy payload method returned no status | Implemented; backpack path exercised in emulator | Record a location as serviced only when its actuator command was published |
| Deployment-disabled payload bypass | Legacy mission always waited for and reset payload servos | Implemented | Permit detector/navigation tests without freezing on an intentionally absent payload controller |
| Confirmed climb coordinates | Legacy diversion used one fixed X/Y throughout | Implemented; exercised in emulator | Climb vertically from the filtered deployment position before returning to the interrupted route |
| Direct `~roi` compatibility | Legacy unclassified ROI diversion | Implemented; runtime verification pending | Preserve safe movement-only diversion without inventing a target class or payload choice |
| Explicit person/backpack injection | Legacy fixed `fake_roi` movement command | Implemented | Exercise class-specific diversion and deployment at a nearby obstacle-free location without pretending the detector produced evidence |
| `/objects/injected_test_target` world-model input | No direct legacy equivalent | Implemented and visibly labelled `TEST` | Display explicit test locations through the normal persistent marker topic while keeping them out of live detector confirmation |
| Mission-start ArUco, target and coverage reset | Legacy state could persist between runs | Implemented | Begin each mission with clear persistent map state while warning rather than blocking if a reset service is unavailable |
| Emulator re-arm at mission start | Emulator remained disarmed after a completed landing, causing a same-session restart to be rejected by SPAR | Implemented; real flight excluded | Restore the emulator's armed state before waiting for SPAR while preserving manual arming for the real aircraft |
| Height-scaled camera-coverage trail | Fixed 1.4 m by 2.3 m RViz rectangles | Implemented | Use published OAK-D intrinsics and current height while retaining the existing coverage topic; begin accumulation at the first sweep waypoint so takeoff and initial transit are excluded |
| Resettable drone snail trail | RViz-local Odometry history | Implemented | Publish mission-owned travelled positions that can be cleared reliably at each mission start |
| COCO 80-class filter/remap | Legacy detector already published application IDs | Implemented | Keep the decoder compatible with the supplied model while exposing only person 0 and backpack 1 downstream |
| 8 FPS sensor/inference setting and newest-frame overlay | Legacy pipeline used 20 FPS and could render a backlog | Implemented | Reduce OAK/RPi load and keep the operator display current without changing detector-message publication |
| ArUco and target labels in RViz | Legacy ArUco cubes had no text; targets had no independent world model | Implemented | Show object identity and XYZ while keeping RViz presentation separate from mission decisions |
| GCS target and mission announcements | Announcers previously applied a separate target policy and retained mission-event history between runs | Implemented; GCS deployment pending | Speak the mission's existing target decisions, reset deduplication at mission start and retain independent ArUco reporting |

## Change-entry template

For each implemented navigation change, record:

1. the affected class, method or block;
2. the legacy behaviour;
3. the Take #2 behaviour;
4. why the change belongs to target handling;
5. any new ROS topic, parameter or dependency;
6. the verification performed.
