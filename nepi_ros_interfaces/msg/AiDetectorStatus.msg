# Image Detection Model Status Message

string MODEL_STATE_LOADING=Loading
string MODEL_STATE_LOADED=Loaded
string MODEL_STATE_WAITING=Waiting
string MODEL_STATE_WAITING=Listening
string MODEL_STATE_RUNNING=Running
string MODEL_STATE_RUNNING=Sleeping

string SLEEP_SUSPEND_UNTIL_WOKEN=-1

string ALL_IMAGES=All

string name

string namespace
bool enabled
string state

bool sleep_enabled
int32 sleep_suspend_sec
int32 sleep_run_sec

bool has_img_tiling
bool img_tiling

bool overlay_labels
bool overlay_clf_name


float32 threshold
float32 max_rate_hz

string[] image_detector_namespaces
bool[] image_detector_states
string[] image_source_topics


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






