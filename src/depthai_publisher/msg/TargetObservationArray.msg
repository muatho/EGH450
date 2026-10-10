# Capture time and optical frame of the source image.
std_msgs/Header header

# Coordinate frame used by valid map positions.
string map_frame_id

uint32 image_width
uint32 image_height

depthai_publisher/TargetObservation[] observations
