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

## Implemented navigation-script changes

### Target evidence window scaffold

`TargetEvidenceWindow` has been added above the existing `Guidance` class and
is instantiated in `Guidance.__init__()`.

The helper:

- retains the most recent N observation frames in a bounded `deque`;
- applies a confidence threshold for each target class;
- ignores observations without a valid map position;
- associates only same-class observations within the matching radius;
- counts at most one associated observation from each frame;
- confirms a cluster after support in at least M retained frames;
- returns the median confidence and median map position of the supporting
  observations;
- ranks simultaneous candidates by supporting-frame count and then median
  confidence.

Current provisional configuration:

- M = 3 detected frames required for confirmation
- N = 5 recent frames retained
- minimum confidence = 0.60 for both classes
- spatial matching radius = 0.75 m

These values are inherited where possible from the legacy target logic and are
not considered tuned. They should later be reviewed using the offline
evaluator.

`callback_object_observations()` now supplies every eligible observation frame
to the helper. A confirmed result retains the established legacy hand-off into
`callback_inspect_roi()`; Take #2 has not replaced Breadcrumb or SPAR movement.

Verification: the edited mission script passes Python compilation.

### Target-observation interface

The mission now subscribes to `/objects/target_observations`. Each observation
contains class, confidence, image box, image dimensions and an explicitly valid
or invalid map position. Empty arrays are still published when a frame contains
no detections, allowing missed frames to enter the M-of-N window.

The object-position estimator continues publishing `/objects/observations` for
existing consumers. Its target output uses the aircraft pose nearest the source
image timestamp and estimates depth from the median of a fixed 7 x 7 patch at
the box centre.

The OAK frame sequence number is copied into the RGB, compressed-image, camera
information, aligned-depth and YOLO headers so corresponding sensor products
retain a shared frame identity.

## Current integration limitations

The initial M-of-N hand-off is connected, but it is not yet a complete target
mission flow:

- `pending_payload_class_id` is currently cleared as soon as the legacy ROI
  callback returns, although deployment occurs later;
- a confirmed location is added to `serviced_target_locations` before
  deployment has succeeded;
- the target location is not yet updated from fresh observations during the
  diversion;
- final image centring, stability reassessment and the three-second deadline
  are not yet implemented;
- reacquisition, candidate rejection, test-target injection and optional GCS
  veto have not yet been reintroduced.

These are recorded limitations, not approved final behaviour.

## Agreed implementation order

1. Retain the target class throughout diversion and deployment.
2. Mark a target serviced only after successful deployment.
3. Verify the M-of-N-to-legacy-diversion path in the emulator.
4. Update the filtered target location from fresh detections during diversion,
   without allowing unstable observations to move the destination continually.
5. Add final centring and reassessment with a fixed three-second deadline.
6. Reintroduce target markers, test injection, candidate rejection or
   blacklisting, optional GCS veto and simple reacquisition individually.
7. Add optional operator-requested preflight diagnostics as the final layer.

## Legacy-to-Take #2 mapping

| Take #2 addition or replacement | Legacy location | Status | Reason |
| --- | --- | --- | --- |
| `TargetEvidenceWindow` | Candidate counting inside `callback_object_observations()` | Implemented and connected | Provide bounded M-of-N and spatially consistent confirmation while keeping the existing diversion hand-off |
| `TargetObservationArray` subscription | Legacy `Detection3DArray` subscription | Implemented | Preserve image evidence and map validity together without changing the raw detector output |
| Confirmed target hand-off | `callback_inspect_roi()` | Preserved | Leave Breadcrumb/SPAR diversion movement under the recognisable legacy mission |

## Change-entry template

For each implemented navigation change, record:

1. the affected class, method or block;
2. the legacy behaviour;
3. the Take #2 behaviour;
4. why the change belongs to target handling;
5. any new ROS topic, parameter or dependency;
6. the verification performed.
