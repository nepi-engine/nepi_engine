# This message provides the Numurus IDX status, as defined in the IDX interface standard. It is generic to
# all IDX sensor nodes in the system. The various control values reported in this message are deliberately
# generic - each IDX sensor defines what they mean with regards to its specific data formats. They provide
# a minimal set of tweaks to reduce complexity for operators.

# Device Identifiers
string device_name
string sensor_name
string identifier
string serial_num
string hw_version
string sw_version

# URL to RTSP video stream if exists
string rtsp_url
string rtsp_username
string rtsp_password

# user updatable device name
string user_name

# In standby or not
bool standby

# Following provide the range limits in meters for the device (if available)
float32 min_range_m
float32 max_range_m

# Data Controls
float32 resolution_ratio
string resolution_current

float32 framerate_ratio
string[] data_products
float32[] framerates

bool auto_adjust_enabled

# The contrast field indicates the 0.0 - 1.0 contrast setting of the device
float32 contrast_ratio

# The brightness field indicates the 0.0 - 1.0 brightness setting of the device
float32 brightness_ratio

# The thresholding field indicates the 0.0 - 1.0 thresholding setting of the device
float32 threshold_ratio

# The range_window field provides the start and stop range as currently configured for the device
# Each range is expressed as a multiplier on the sensor's max range (0.0 - 1.0)
RangeWindow range_window_ratios

string[] avail_pantilt_topics
# If Device is Pan and Tilt Mounted
bool pantilt_mounted
# Selected Pan Tilt Device and NavPose Topic
string sel_pantilt_name
string sel_pantilt_device_topic
string sel_pantilt_navpose_topic
bool sel_pantilt_connected
# Meters from center base of pan tilt to hieight tilt axis 
# Measured in upright position, inversions handled downstream
float32 pantilt_height_to_tilt_axis 
# Meters x,y,z from pantilt tilt center axis in ENU frame
# Measured in upright position, inversions handled downstream
float32 pantilt_x_from_tilt_axis
float32 pantilt_y_from_tilt_axis
float32 pantilt_z_from_tilt_axis

# Input Transform for data from sensor's native sensor frame ( or PanTilt center base frame) to the nepi_center_frame 
nepi_sdk_interfaces/Frame3DTransform frame_3d_transform

# Output transform into which 3D data (navpose,pointclouds...) are published
# Options ['nepi_base_frame','sensor_frame','map']
string frame_3d



