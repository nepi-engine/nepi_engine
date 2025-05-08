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
