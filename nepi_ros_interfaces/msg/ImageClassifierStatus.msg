# Image Classifier Status Message

string CLASSIFIER_STATE_LOADING=Loading
string CLASSIFIER_STATE_LOADED=Loaded
string CLASSIFIER_STATE_WAITING=Waiting
string CLASSIFIER_STATE_RUNNING=Running

string classifier_name
bool classifier_enabled
string classifier_state
string[] classifier_classes

string[] img_topics
bool img_tiling
bool overlay_labels
bool overlay_clf_name
bool overlay_img_name
string img_topic
bool img_connected

float32 threshold
float32 max_rate_hz



