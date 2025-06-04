# NavPose Mgr Status Message

# These may change based on current confuration
# Value is updated after first data is recieved
bool has_heading
bool has_position
bool has_orientation
bool has_location
bool has_altitude
bool has_depth


string[] available_location_topics
string location_topic
float32 location_avg_rate
float32 location_last_time
Frame3DTransform location_transform

string[] available_heading_topics
string heading_topic
float32 heading_avg_rate
float32 heading_last_time
Frame3DTransform heading_transform

string[] available_orientation_topics
string orientation_topic
float32 orientation_avg_rate
float32 orientation_last_time
Frame3DTransform orientation_transform

string[] available_position_topics
string position_topic
float32 position_avg_rate
float32 position_last_time
Frame3DTransform position_transform

string[] available_altitude_topics
string altitude_topic
float32 altitude_avg_rate
float32 altitude_last_time
Frame3DTransform altitude_transform

string[] available_depth_topics
string depth_source_topic
float32 depth_avg_rate
float32 depth_last_time
Frame3DTransform depth_transform


bool publishing


float32 pub_rate




