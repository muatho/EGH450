int32 class_id
float64 confidence

# Pixel coordinates in the source image.
float64 bbox_center_x
float64 bbox_center_y
float64 bbox_width
float64 bbox_height

# The position is usable only when map_position_valid is true.
bool map_position_valid
geometry_msgs/Point map_position
