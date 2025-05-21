# This is an optional status message for image publishers

bool publishing
float32 last_pub_sec
float32 fps

string encoding
int32 width
int32 height

# Following provide the range limits in meters for the device (if available)
float32 min_range_m
float32 max_range_m

string frame_id

float32 get_latency_time
float32 pub_latency_time
float32 process_time
float32 avg_fps

bool image_pub_enabled
int32 max_image_pub_rate
######################################
# Depthmap Render Controls if image publishing is enabled

# Each range is expressed as a multiplier on the pointcloud's min/max range (0.0 - 1.0)
RangeWindow range_ratios
