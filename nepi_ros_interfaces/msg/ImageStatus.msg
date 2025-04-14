# This is an optional status message for image publishers

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

string depth_map_topic
string pointcloud_topic

float32 get_latency_time
float32 pub_latency_time
float32 process_time



