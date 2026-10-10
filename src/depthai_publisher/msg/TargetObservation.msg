# One untracked, per-frame target observation.
#
# The containing TargetObservationArray supplies the common source-image
# timestamp, frame ID and sequence number.

int32 class_id
float32 confidence

# Bounding box in original RGB-image pixel coordinates.
float32 bbox_center_x
float32 bbox_center_y
float32 bbox_width
float32 bbox_height
uint32 image_width
uint32 image_height

# Estimated object position in the map frame.
geometry_msgs/Point map_position

# Robust depth information used to estimate the map position.
float32 depth_median_metres
float32 depth_mad_metres
uint32 valid_depth_samples
float32 valid_depth_fraction
