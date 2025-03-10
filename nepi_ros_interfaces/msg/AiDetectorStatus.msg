# Image Detection Model Status Message

string MODEL_STATE_LOADING=Loading
string MODEL_STATE_LOADED=Loaded
string MODEL_STATE_WAITING=Waiting
string MODEL_STATE_WAITING=Listening
string MODEL_STATE_RUNNING=Running
string ALL_IMAGES=All

string name

string namespace
bool enabled
string state

bool using_latest_img

bool has_img_tiling
bool img_tiling

bool overlay_labels
bool overlay_clf_name
bool overlay_img_name


float32 threshold
float32 max_rate_hz

string[] image_source_topics
string[] image_detect_namespaces
bool[] images_connected
float32[] preprocess_times
float32[] detect_times
float32[] postprocess_times

bool image_selected
bool image_connected
float32 avg_preprocess_time
float32 avg_detect_time
float32 avg_postprocess_time






