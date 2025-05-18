# This message contains status information for the navpose publisher 

std_msgs/Header header
string frame_3d
string frame_altitude

float32 geoid_height_meters

bool has_heading
float32 time_heading
float32 heading_deg

bool has_orientation
float32 time_orientation
# Orientation Degrees in selected 3d frame (roll,pitch,yaw)
float32 roll_deg
float32 pitch_deg
float32 yaw_deg

bool has_position
float32 time_position
# Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
float32 x_m
float32 y_m
float32 z_m

bool has_location
float32 time_location
# Global Location in set altitude frame (lat,long,alt) with alt in meters
float32 lat
float32 long

bool has_altitude
float32 time_altitude
float32 altitude_m

bool has_depth
float32 time_depth
float32 depth_m


