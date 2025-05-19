# This is an optional status message for pointcloud publishers

bool publishing
float32 last_pub_sec
float32 fps

bool has_rgb
bool has_intensity

int32 point_count

int32 width
int32 height
int32 depth

string frame_id

float32 get_latency_time
float32 pub_latency_time
float32 process_time
float32 avg_fps

######################################
# This message details pointcloud processing status
# Range clipping on pointcloud enabled

bool controls_enabled

bool clip_enabled
string[] clip_options
string clip_selection

# Range limits in meters for the clipping
RangeWindow clip_meters

# Clip bounding box topic
string clip_target_topic

# Voxel Downsample Size, val 0 skips process
float32 voxel_downsample_size_m

# Uniform Downsample Size, val 0 skips process
int32 uniform_downsample_points

# Outlier Removal K Points, val 0 skips process
int32 outlier_k_points


######################################
# The range_window field provides the start and stop range as currently configured for the device


# Standard Image Size Opions string list
string standard_image_sizes

# Render Image Size
int32 image_width
int32 image_height

# Each range is expressed as a multiplier on the sensor's max range (0.0 - 1.0)
RangeWindow range_clip_ratios


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

# Render Enable
bool render_enable