# This message provides Controls for Nepi Data Sonar Class
# Set all fields if sending to 'set_all_controls' topic
# Set topic_name and specific control value for specific control topics

# Data Product Name (color_2d_img, bw_2d_img, depth_map, pointcloud...)
string data_product

# Enable or Disable controls
bool controls_enable

# Enable or Disable auto adjusting brightness, contrast, and threshold control
bool auto_adjust

# The contrast field indicates the 0.0 - 1.0 contrast setting of the device
float32 contrast

# The brightness field indicates the 0.0 - 1.0 brightness setting of the device
float32 brightness

# The thresholding field indicates the 0.0 - 1.0 thresholding setting of the device
float32 thresholding

# The range_window field provides the start and stop range for 
# devices/data_products with range support
# Each range is expressed as a multiplier on the sensor's max range (0.0 - 1.0)
RangeWindow range_window







