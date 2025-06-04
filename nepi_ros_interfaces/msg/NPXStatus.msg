# This message contains status information for the NPX devices 


bool has_location
bool set_as_locaition_source

bool has_heading
bool set_as_heading_source

bool has_orientation
bool set_as_orienation_source

bool has_position
bool set_as_position_source

bool has_altitude
bool set_as_altitude_source

bool has_depth
bool set_as_depth_source

string frame_3d

# Provides the transform to the system 3D frame 'nepi_frame'
# (Not applied by NPX driver)
Frame3dTransforms nepi_frame_3d_transform

# Actual data will be in the nepi standard frames 
string source_frame_nav
string source_frame_altitude
string source_frame_depth

float32 update_rate



