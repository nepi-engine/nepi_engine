# This message provides Controls for Numurus IDX Devices.
# Set all fields if sending to 'set_controls' topic
# Set topic_name and specific control value for specific control topics

# Enable or Disable IDX controls
bool controls_enable

float32 resolution_ratio
float32 framerate_ratio



# Enable or Disable IDX auto adjusting brightness, contrast, and threshold control
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







