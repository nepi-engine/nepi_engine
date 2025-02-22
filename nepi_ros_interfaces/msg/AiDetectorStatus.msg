# Image Detection Model Status Message

string MODEL_STATE_LOADING=Loading
string MODEL_STATE_LOADED=Loaded
string MODEL_STATE_WAITING=Waiting
string MODEL_STATE_RUNNING=Running

string model_name
string model_type
string model_description
int32 img_height
int32 img_width
string[] model_classes

string model_namespace

bool model_enabled
string model_state

string[] image_source_topics
string[] image_namespaces

string image_topic
bool image_connected

bool img_tiling
bool overlay_labels
bool overlay_clf_name
bool overlay_img_name

float32 threshold
float32 max_rate_hz



