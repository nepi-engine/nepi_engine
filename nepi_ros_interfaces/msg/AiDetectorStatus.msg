# Image Detection Model Status Message

string MODEL_STATE_LOADING=Loading
string MODEL_STATE_LOADED=Loaded
string MODEL_STATE_WAITING=Waiting
string MODEL_STATE_RUNNING=Running
string ALL_IMAGES=All

string model_name
AiModelInfo model_info

string namespace
bool enabled
string state

string[] image_source_topics
string[] image_detect_namespaces

string detection_image_topic
bool detection_image_connected

bool img_tiling
bool overlay_labels
bool overlay_clf_name
bool overlay_img_name

float32 threshold
float32 max_rate_hz



