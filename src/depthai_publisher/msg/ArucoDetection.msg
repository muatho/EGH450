# One ArUco observation in the source image's pixel coordinates.
int32 id
geometry_msgs/Point32[4] corners

# PnP can be unavailable or rejected even though the marker border and ID were
# detected. Consumers must check pose_valid before using these fields.
bool pose_valid
geometry_msgs/Vector3 rvec
geometry_msgs/Vector3 tvec
float64 reprojection_error
