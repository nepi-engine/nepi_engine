# This message contains status information for the NPX devices 

# Device Identifiers
string device_name
string device_id
string identifier
string serial_num
string hw_version
string sw_version

string data_source_description
string data_ref_description
string device_mount_description

float32 update_rate

string frame_3d

bool supports_transform_updates

# Transform from device native ref frame to the nepi_frame,
nepi_interfaces/Frame3DTransform frame_3d_transform

# Actual data will be in the nepi standard frames 
string source_frame_nav
string source_frame_altitude
string source_frame_depth


bool has_location
bool set_as_location_source

bool has_heading
bool set_as_heading_source

bool has_orientation
bool set_as_orientation_source

bool has_position
bool set_as_position_source

bool has_altitude
bool set_as_altitude_source

bool has_depth
bool set_as_depth_source





