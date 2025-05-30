# This message contains status information for the NPX devices 

float32 update_rate
string frame_id
string frame_3d
string frame_altitude

bool has_heading
bool has_position
bool has_orientation
bool has_location
bool has_altitude
bool has_depth

bool has_gps_pub
bool set_as_gps_source

bool has_elavation_pub
bool set_as_elevation_source

bool has_pose_pub
bool set_as_pose_source

bool has_heading_pub
bool set_as_heading_source

# Input Transform for data from sensor's native sensor frame to the nepi_center_frame 
nepi_ros_interfaces/Frame3DTransform frame_transform



