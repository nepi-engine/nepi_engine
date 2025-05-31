# This is an optional status message for image publishers


# Image data product type
# SUPPORTED_DATA_PRODUCTS = ['image','color_image','bw_image',
                            'intensity_map','depth_map','pointcloud']
string data_product_type

bool publishing
float32 last_pub_sec
float32 fps

string encoding
int32 width
int32 height

string frame_id

# Render Controls

# The resolution field indicates the 0.0 - 1.0 brightness setting of the device
float32 resolution_ratio

# Enable or Disable IDX auto adjusting brightness, contrast, and threshold control
bool auto_adjust_enabled

# The contrast field indicates the 0.0 - 1.0 contrast setting of the device
float32 contrast_ratio

# The brightness field indicates the 0.0 - 1.0 brightness setting of the device
float32 brightness_ratio

# The thresholding field indicates the 0.0 - 1.0 thresholding setting of the device
float32 thresholding_ratio

# The range_window field provides the start and stop range as currently configured for the device
# Each range is expressed as a multiplier on the sensor's max range (0.0 - 1.0)
RangeWindow range_window

# The zoom field 0.0 - 1.0 
float32 zoom_ratio

# The pan fields 0.0 - 1.0 
float32 pan_left_right_ratio
float32 pan_up_down_ratio

# window pixel ratios 0.0 - 1.0 x_min,x_max,y_min,y_max
ImageWindow window_ratios

# The rotate field 0.0 - 1.0 
float32 rotate_ratio

# The tilt field 0.0 - 1.0 
float32 tilt_ratio


# Overlay Controls
bool overlay_img_name
bool overlay_date_time
bool overlay_nav
bool overlay_pose
string[] base_overlay_list
string[] add_overlay_list


# Render Stats
float32 get_latency_time
float32 pub_latency_time
float32 process_time
float32 avg_fps



