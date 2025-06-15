# This message contains status information for the NPX devices 

# Device Identifiers
string device_name
string sensor_name
string identifier
string serial_num
string hw_version
string sw_version

string data_ref_description

float32 update_rate

string frame_3d

# Provides the transform to the system 3D frame 'nepi_frame'
# (Not applied by NPX driver)
Frame3DTransform nepi_frame_3d_transform

bool include_transform_enabled

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





