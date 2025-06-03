# This message contains status information for the system navpose solution output
# The system excepts navpose_data messages
# All navpose input data is expected to be in the following frames:
#   Frame_ID -> nepi_frame
#   Frame_3D -> ENU
#   Frame_Altitude -> WGS84
# navpose_data publishers should do transforms to these frames before publishing
# nepi_frame solution data is provide as a published navpose_data &
#  navpose_solution topics on the system's base namespace
# frame adjusted solution data is available through the navpose_data_query &
#  navpose_solution_query services on the systems base


# Current topic for the lat_long inputs
bool lat_long_fixed
#else
string lat_long_topic
time last_lat_long_
float32 lat_long__rate

# Current topic for the orientation (pose, twist) inputs
string orientation_topic
time last_orientation
float32 orientation_rate




# Current topic for the heading input
string heading_topic

# Sys. clock time when most recent odometry update received
time last_heading

# Rate (Hz) of heading updates
float32 heading_rate

# Transform from src to output (includes names of source and target frame ID)
geometry_msgs/TransformStamped transform

# Rotational adjsutment applied just to heading
float32 heading_offset

# GPS clock sync. is enabled
bool gps_clock_sync_enabled
