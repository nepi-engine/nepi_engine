# This is an optional status message for pointcloud publishers

bool publishing
float32 last_pub_sec
float32 fps

bool has_rgb
bool has_intensity

int32 width_m
int32 height_m
int32 depth_m

int32 point_count

string frame_id

float32 get_latency_time
float32 pub_latency_time
float32 process_time
float32 avg_fps

bool image_pub_enabled
int32 max_image_pub_rate
######################################
# Pointcloud Render Controls if image publishing is enabled

# Standard Image Size Opions string list
string[] standard_image_sizes

# Render Image Size
int32 image_width
int32 image_height

# The range_window field provides the start and stop range as currently configured for the device
# Each range is expressed as a multiplier on the sensor's max range (0.0 - 1.0)
nepi_sdk_interfaces/RangeWindow range_ratios

# Zoom Ratio 0.0 - 1.0
float32 zoom_ratio

# Rotate Ratio 0.0 - 1.0
float32 rotate_ratio

# Tilt Ratio 0.0 - 1.0
float32 tilt_ratio

# Camera Field of View
int32 camera_fov

# Camera View in meters (x,y,z)
geometry_msgs/Vector3 camera_view

# Camera Position in meters (x,y,z)
geometry_msgs/Vector3 camera_position

# Camera Rotation -1,0,1 options for (x,y,z)
geometry_msgs/Vector3 camera_rotation

bool white_background


