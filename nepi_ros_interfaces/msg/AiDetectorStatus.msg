# Image Detection Model Status Message

string MODEL_STATE_LOADING=Loading
string MODEL_STATE_LOADED=Loaded
string MODEL_STATE_WAITING=Waiting
string MODEL_STATE_LISTENING=Listening
string MODEL_STATE_RUNNING=Running
string MODEL_STATE_SLEEPING=Sleeping

string MAX_IMG_RATE_NO_PUB=-1

string ALL_IMAGES=All

string name

string namespace
bool enabled
string state

float32 max_proc_rate_hz
float32 max_img_rate_hz

string[] available_classes
string[] selected_classes
std_msgs/ColorRGBA[] selected_classes_colors

float32 threshold

bool overlay_labels
bool overlay_clf_name
bool overlay_img_name

string[] selected_img_topics
string[] image_source_topics
string[] image_detector_namespaces
bool[] images_connected
float32[] image_latency_times
float32[] detect_latency_times
float32[] preprocess_times
float32[] detect_times

bool image_selected
bool image_connected

float32 image_latency_time
float32 detect_latency_time
float32 preprocess_time
float32 detect_time
float32 avg_rate_hz






