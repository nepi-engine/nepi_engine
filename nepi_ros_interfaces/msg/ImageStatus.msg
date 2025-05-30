# This is an optional status message for image publishers

string IMAGE=image
string SONAR=sonar
string DEPTH_MAP=depth_map
string POINTCLOUD=pointcloud
string LASER_LINE=laser_line


# Image data type ['image','sonar','depthmap','pointcloud','laserline']
string data_type

bool publishing
float32 last_pub_sec
float32 fps

string encoding
int32 width
int32 height
string frame_id

bool overlay_img_name
bool overlay_date_time
bool overlay_nav
bool overlay_pose
string[] base_overlay_list
string[] add_overlay_list



float32 get_latency_time
float32 pub_latency_time
float32 process_time
float32 avg_fps

# Current Controls Feedback
bool controls_enabled
bool auto_adjust_enabled
float32 contrast_ratio
float32 brightness_ratio
float32 threshold_ratio


