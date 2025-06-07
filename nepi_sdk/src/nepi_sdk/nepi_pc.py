#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI pointcloud utility functions include
# 1) Pointcloud conversion functions
# 2) Pointcloud filter functions
# 3) Pointcloud manipulation functions
# 4) Pointcloud rendering functions
# 5) Pointcloud saving functions


import numpy as np
import ros_numpy
import os
os.environ['EGL_PLATFORM'] = 'surfaceless'   # Ubuntu 20.04+
import open3d as o3d
import math
import cv2

from nepi_sdk import open3d_ros_helper 

from sensor_msgs.msg import PointCloud2

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_pc"
logger = Logger(log_name = log_name)

VERBOSITY_LEVELS = ["Debug","Error","Info","Warning"]

def get_verbosity_level():
    return o3d.utility.get_verbosity_level()

def set_verbosity_level(level = "Error"):
    if level in VERBOSITY_LEVELS:
        eval("o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel." + level + ")")
    return get_verbosity_level()


def get_pointcloud_publisher_namespaces(name):
    msg_type = 'sensor_msgs/PointCloud2'
    return nepi_sdk.find_topics_by_msg(msg_type)


###########################################
### Pointcloud conversion functions

def create_empty_o3dpc():
    return o3d.t.geometry.PointCloud()

def rospc_to_o3dpc(ros_pc, remove_nans=False):
    ''' Convert ROS PointCloud2 to Open3D PointCloud
    
    Args: 
        rospc (sensor.msg.PointCloud2): ROS PointCloud2 message
        remove_nans (bool): If true, ignore the NaN points
    
    Returns: 
        o3dpc (o3d.geometry.PointCloud): Open3D PointCloud
    '''
    o3d_pc = open3d_ros_helper.rospc_to_o3dpc(ros_pc, remove_nans)
    return o3d_pc
    

def o3dpc_to_rospc(o3d_pc, stamp=None, frame_id=None):
    ''' convert open3d point cloud to ros point cloud
    Args:
        o3dpc (o3d.geometry.PointCloud): open3d point cloud
        frame_id (string): frame id of ros point cloud header
        stamp (rospy.Time): time stamp of ros point cloud header
    Returns:
        rospc (sensor.msg.PointCloud2): ros point cloud message
    '''

    ros_pc = open3d_ros_helper.o3dpc_to_rospc(o3d_pc, stamp, frame_id)
    return ros_pc
    
    
def rosimg_to_o3dimg(ros_img_msg):
    np_array = ros_numpy.image.image_to_numpy(ros_img_msg)  
    o3d_img = o3d.geometry.Image(np_array)
    return o3d_img

def o3dimg_to_rosimg(o3d_img, stamp=None, frame_id=None, encoding="rgb8"):
    np_array = np.asarray(o3d_img)
    ros_img_msg = ros_numpy.image.numpy_to_image(np_array, encoding)
    
    if stamp is None:
        ros_img_msg.header.stamp = nepi_sdk.get_msg_stamp()
    else:
        ros_img_msg.header.stamp = stamp
    
    if frame_id is not None:
        ros_img_msg.header.frame_id = frame_id
    else:
        ros_img_msg.header.frame_id="nepi_center_frame"
    return ros_img_msg

def o3dimg_to_cv2img(o3d_img):
    np_array = np.asarray(o3d_img)
    cv2_image = np_array # cv2 images are just numpy arrays
    return cv2_image

def cv2img_to_o3dimg(cv2_image):
    np_array = np.asarray(cv2img[:,:])
    o3d_img = o3d.geometry.Image(np_array)
    return o3d_img
    
###########################################
### Pointcloud info functions    

def get_min_range(o3d_pc):
    min_range = get_min_ranges(o3d_pc)
    min_distance = np.linalg.norm(min_range)
    return min_distance
    
def get_max_range(o3d_pc):
    max_range = get_max_ranges(o3d_pc)
    max_distance = np.linalg.norm(max_range)
    return max_distance
   
def get_min_ranges(o3d_pc):
    min_bounds = np.asarray(o3d_pc.get_min_bound())
    return (min_bounds)
    
def get_max_ranges(o3d_pc):
    max_bounds = np.asarray(o3d_pc.get_max_bound()) 
    return (max_bounds)	
	


###########################################
### Pointcloud filter functions    

def apply_pass_through_filter(o3d_pc, x_range, y_range, z_range):
    ''' apply 3D pass through filter to the open3d point cloud
    Args:
        o3dpc (o3d.geometry.PointCloud): open3d point cloud
        x_range (list): list of [x_min, x_maz]
        y_range (list): list of [y_min, y_maz]
        z_range (list): list of [z_min, z_max]
    Returns:
        o3dpc (o3d.geometry.PointCloud): filtered open3d point cloud
    some codes from https://github.com/powersimmani/example_3d_pass_through-filter_guide
    '''
    o3d_pc = open3d_ros_helper.apply_pass_through_filter(o3d_pc, x_range, y_range, z_range)
    return o3d_pc
    
###########################################
### Pointcloud manipulation functions

def do_transform_point(o3d_pc, transform_stamped):
    ''' transform a input cloud with respect to the specific frame
        open3d version of tf2_geometry_msgs.do_transform_point
    Args: 
        o3dpc (o3d.geometry.PointCloud): open3d point cloud
        transform_stamped (geometry_msgs.msgs.TransformStamped): transform to be applied 
    Returns:
        o3dpc (o3d.geometry.PointCloud): transformed open3d point cloud
    '''
    o3d_pc = open3d_ros_helper.do_transform_point(o3d_pc, transform_stamped)
    return o3dpc
    


def crop_with_2dmask(o3d_pc, mask, K=None):
    ''' crop open3d point cloud with given 2d binary mask
    Args: 
        o3dpc (o3d.geometry.PointCloud): open3d point cloud
        mask (np.array): binary mask aligned with the point cloud frame shape of [H, W]
        K (np.array): intrinsic matrix of camera shape of (4x4)
        if K is not given, point cloud should be ordered
    Returns:
        o3dpc (o3d.geometry.PointCloud): filtered open3d point cloud
    '''
    o3d_pc = open3d_ros_helper.crop_with_2dmask(o3d_pc, mask, K=None)
    return o3d_pc

def range_clip_spherical(o3d_pc, range_clip_min_range_m, range_clip_max_range_m):
    points = np.asarray(o3d_pc.points)
    distances = np.linalg.norm(points, axis=1)
    clipped_points = (distances >= range_clip_min_range_m) & (distances <= range_clip_max_range_m)
    o3d_pc_has_colors = o3d_pc.colors
    if o3d_pc_has_colors:
        has_colors = True
        o3d_pc_colors = np.asarray(o3d_pc.colors)
        o3d_pc_colors = o3d_pc_colors[clipped_points]    
    
    o3d_pc_points = points[clipped_points]
    
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(o3d_pc_points)
    if o3d_pc_has_colors:
        o3d_pc.colors = o3d.utility.Vector3dVector(o3d_pc_colors)
    return o3d_pc

def range_clip_x_axis( o3d_pc, range_clip_min_range_m, range_clip_max_range_m):
    ''' Clip input Open3d PointCloud on x-axis based on min range meters and max range meters
    
    Args:
      o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
      range_clip_min_range_m (float): minimum clip range in meters
      range_clip_max_range_m (float): maximum clip range in meters
     
    Returns:
      o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
    '''
    
    o3d_pc_points = np.asarray(o3d_pc.points)
    # Check if pointcloud has color
    o3d_pc_has_colors = o3d_pc.colors
    if o3d_pc_has_colors:
      has_colors = True
      #print("o3d_pc has colors")
      ### Need to Add color data as well if it has color data
      o3d_pc_colors = np.asarray(o3d_pc.colors)
      o3d_pc_colors = o3d_pc_colors[(o3d_pc_points[:, 0] >= range_clip_min_range_m) & (o3d_pc_points[:, 0] <= range_clip_max_range_m)]
    o3d_pc_points = o3d_pc_points[(o3d_pc_points[:, 0] >= range_clip_min_range_m) & (o3d_pc_points[:, 0] <= range_clip_max_range_m)]
    # Clear and create new pointcloud      
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(o3d_pc_points)
    if o3d_pc_has_colors:
      #print("adding colors to new o3d_pc")
      o3d_pc.colors = o3d.utility.Vector3dVector(o3d_pc_colors)
    return o3d_pc
    
def range_clip_y_axis(o3d_pc, range_clip_min_range_m, range_clip_max_range_m):
    ''' Clip input Open3d PointCloud on y-axis based on min range meters and max range meters
    
    Args:
      o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
      range_clip_min_range_m (float): minimum clip range in meters
      range_clip_max_range_m (float): maximum clip range in meters
     
    Returns:
      o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
    '''
    
    o3d_pc_points = np.asarray(o3d_pc.points)
    # Check if pointcloud has color
    o3d_pc_has_colors = o3d_pc.colors
    if o3d_pc_has_colors:
      has_colors = True
      #print("o3d_pc has colors")
      ### Need to Add color data as well if it has color data
      o3d_pc_colors = np.asarray(o3d_pc.colors)
      o3d_pc_colors = o3d_pc_colors[(o3d_pc_points[:, 1] >= range_clip_min_range_m) & (o3d_pc_points[:, 1] <= range_clip_max_range_m)]
    o3d_pc_points = o3d_pc_points[(o3d_pc_points[:, 1] >= range_clip_min_range_m) & (o3d_pc_points[:, 1] <= range_clip_max_range_m)]
    # Clear and create new pointcloud      
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(o3d_pc_points)
    if o3d_pc_has_colors:
      #print("adding colors to new o3d_pc")
      o3d_pc.colors = o3d.utility.Vector3dVector(o3d_pc_colors)
    return o3d_pc

def range_clip_z_axis(o3d_pc, range_clip_min_range_m, range_clip_max_range_m):
    ''' Clip input Open3d PointCloud on z-axis based on min range meters and max range meters
    
    Args:
      o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
      range_clip_min_range_m (float): minimum clip range in meters
      range_clip_max_range_m (float): maximum clip range in meters
     
    Returns:
      o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
    '''
    
    o3d_pc_points = np.asarray(o3d_pc.points)
    # Check if pointcloud has color
    o3d_pc_has_colors = o3d_pc.colors
    if o3d_pc_has_colors:
      has_colors = True
      #print("o3d_pc has colors")
      ### Need to Add color data as well if it has color data
      o3d_pc_colors = np.asarray(o3d_pc.colors)
      o3d_pc_colors = o3d_pc_colors[(o3d_pc_points[:, 2] >= range_clip_min_range_m) & (o3d_pc_points[:, 2] <= range_clip_max_range_m)]
    o3d_pc_points = o3d_pc_points[(o3d_pc_points[:, 2] >= range_clip_min_range_m) & (o3d_pc_points[:, 2] <= range_clip_max_range_m)]
    # Clear and create new pointcloud      
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(o3d_pc_points)
    if o3d_pc_has_colors:
      #print("adding colors to new o3d_pc")
      o3d_pc.colors = o3d.utility.Vector3dVector(o3d_pc_colors)
    return o3d_pc
    
def angles_to_rotation_matrix( rotation_angles_deg):
    ''' Convert angles (degrees) to 3D Rotation Matrix
        Args:
        rotation_angles_deg (list): Angles in degrees, [x_angle, y_angle, z_angle]
        Returns:
        rotation_matrix (numpy.ndarray[numpy.float64[3, 3]]): 3x3 rotation matrix corresponding to the given angles
    '''  

    angles_rad = np.radians(rotation_angles_deg)
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz(angles_rad)
    return rotation_matrix

def add_pointcloud(o3d_pc, o3d_pc_add):
    o3d_pc += o3d_pc_add
    return o3d_pc
    
def clip_bounding_box( o3d_pc, bounding_box_center, bounding_box_extent, bounding_box_rotation):
    ''' Clip a Open3D Pointcloud with a bounding box
        Args:
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
        bounding_box_center (list): The center of the bounding box in 3D space
        bounding_box_extent (list): The extent of the bounding box along each axis (Meters)
        bounding_box_rotation (list): The rotation of the bounding box for each axis (Degrees)
        Returns:
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
    '''

    bounding_box_center = np.asarray(bounding_box_center).reshape(3, 1)
    bounding_box_extent = np.asarray(bounding_box_extent).reshape(3, 1)
    bounding_box_rotation = angles_to_rotation_matrix(bounding_box_rotation)
    clip_bounding_box = o3d.geometry.OrientedBoundingBox(bounding_box_center, bounding_box_rotation, bounding_box_extent)
    o3d_pc = o3d_pc.crop(clip_bounding_box)
    return o3d_pc

def voxel_down_sampling( o3d_pc, voxel_size_m):
    ''' 
    Down-Sample a Open3D PointCloud using Voxel Grip Downsampling
    Args: 
    o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
    voxel_size_m (float): The size of the voxels in meters
    Returns:
    o3d_pc (o3d.geometry.PointCloud): Down-sampled Open3D PointCloud
    '''

    o3d_pc = o3d_pc.voxel_down_sample(voxel_size_m)
    return o3d_pc

def uniform_down_sampling( o3d_pc, every_k_points):
    ''' Down-Sample a Open3D PointCloud Using Uniform Downsampling with every k point.
        Args: 
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
        every_k_points (int): The sampling rate
        Returns:
        o3d_pc (o3d.geometry.PointCloud): Down-sampled Open3D PointCloud
    '''

    o3d_pc = o3d_pc.uniform_down_sample(every_k_points)
    return o3d_pc

def statistical_outlier_removal( o3d_pc, nb_neighbors, std_ratio):
    ''' Remove statistical outliers from a Open3d PointCloud using statistical outlier removal filter
        Args:
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
        nb_neighbors (int): The number of neighbors to consider for outlier removal
        std_ratio (float): The standard deviation ratio used for determining outliers
        Returns: 
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud with Statistical Outliers Removed
    '''

    o3d_pc = o3d_pc.remove_statistical_outlier(nb_neighbors, std_ratio)
    return o3d_pc

def radius_outlier_removal( o3d_pc, nb_points, search_radius_m):
    ''' Remove Radius Outliers from a Open3D PointCloud using a radius outlier removal filter
        Args: 
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
        nb_points (int): The minimum number of points that the search radius will contain
        search_radius_m (float): Radius of the Sphere that will Count the Neighbors
        Returns:
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud with Radius Outliers Removed
    '''

    o3d_pc = o3d_pc.remove_radius_outlier(nb_points, search_radius_m)
    return o3d_pc

def rotate_pc( o3d_pc, rotation_angles_deg):
    ''' Rotate a Open3D PointCloud around its orgin by specified angles.
        Args:
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
        rotation_angles_deg (list): The rotation angles around the x, y, and z axes in degrees
        Returns:
        o3d_pc (o3d.geometry.PointCloud): Rotated Open3D PointCloud
    '''

    rotation_matrix = angles_to_rotation_matrix(rotation_angles_deg)
    o3d_pc = o3d_pc.rotate(rotation_matrix)
    return o3d_pc

def translate_pc( o3d_pc, translation_vector):
    ''' Translate a Open3D PointCloud by a Specified Translation Vector
        Args:
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
        translation_vector (list): The Translation Vector in Meters
        Returns:
        o3d_pc (o3d.geometry.PointCloud) Translated Open3D PointCloud
    '''
    for i in range(3):
        translation_vector[i] = -1 * translation_vector[i]
    points = np.asarray(o3d_pc.points)
    translated_points = points + translation_vector
    o3d_pc.points = o3d.utility.Vector3dVector(translated_points)
    return o3d_pc


    
###########################################
### Pointcloud registration functions

def p2p_icp_registration(source_cloud, target_cloud, n_points=100, threshold=0.02, \
    relative_fitness=1e-10, relative_rmse=1e-8, max_iteration=500, max_correspondence_distance=500):
    ''' align the source cloud to the target cloud using point-to-point ICP registration algorithm
    Args: 
        source_cloud (o3d.geometry.PointCloud): source open3d point cloud
        target_cloud (o3d.geometry.PointCloud): target open3d point cloud
        for other parameter, go to http://www.o3d.org/docs/0.9.0/python_api/o3d.registration.registration_icp.html
    Returns:
        icp_result (o3d.registration.RegistrationResult): registration result
    '''                        
    [icp_result, evaluation] = open3d_ros_helper.p2p_icp_registration(source_cloud, target_cloud, n_points, threshold, \
    relative_fitness, relative_rmse, max_iteration, max_correspondence_distance)
    return icp_result, evaluation
            
def ppf_icp_registration(source_cloud, target_cloud, n_points=3000, n_iter=100, tolerance=0.001, num_levels=5, scale=0.001):
    ''' align the source cloud to the target cloud using point pair feature (PPF) match
    Args: 
        source_cloud (o3d.geometry.PointCloud): source open3d point cloud
        target_cloud (o3d.geometry.PointCloud): target open3d point cloud
        for other parameter, go to https://docs.opencv.org/master/dc/d9b/classcv_1_1ppf__match__3d_1_1ICP.html
    Returns:
        pose (np.array): 4x4 transformation between source and targe cloud
        residual (float): the output resistration error
    '''
    [pose, residual] = open3d_ros_helper.ppf_icp_registration(source_cloud, target_cloud, n_points=3000, n_iter=100, tolerance=0.001, num_levels=5, scale=0.001)
    return pose, residual



###########################################
### Pointcloud rendering functions

def create_bounding_box( bounding_box_center, bounding_box_extent, bounding_box_rotation, bounding_box_color):
    ''' Create bounding box around a Open3D PointCloud
    
      Args:
        bounding_box_center (list): The center of the bounding box in 3D space
        bounding_box_extent (list): The extent of the bounding box along each axis (Meters)
        bounding_box_rotation (list): The rotation of the bounding box for each axis (Degrees)
      
      Returns: 
        bounding_box_lineset (open3d.geometry.LineSet: A LineSet representing the bounding box
    '''
    
    bounding_box_rotation = angles_to_rotation_matrix(bounding_box_rotation)
    # Create bounding box tensor
    bounding_box_center = o3d.core.Tensor(bounding_box_center, dtype=o3d.core.Dtype.Float32)
    bounding_box_extent = o3d.core.Tensor(bounding_box_extent, dtype=o3d.core.Dtype.Float32)
    bounding_box_rotation = o3d.core.Tensor(bounding_box_rotation, dtype=o3d.core.Dtype.Float32)
    oriented_bounding_box = o3d.t.geometry.OrientedBoundingBox(bounding_box_center, bounding_box_rotation, bounding_box_extent)
    
    # Turn bounding box tensor into legacy bounding box for line set creation
    bounding_box_center = oriented_bounding_box.center.numpy().reshape(3, 1)
    bounding_box_rotation = oriented_bounding_box.rotation.numpy()
    bounding_box_extent = oriented_bounding_box.extent.numpy().reshape(3, 1)
    oriented_bounding_box = o3d.geometry.OrientedBoundingBox(bounding_box_center, bounding_box_rotation, bounding_box_extent)
    
    # Add color if enabled, will be black by default
    oriented_bounding_box.color = bounding_box_color
 
    # Create and return line set for visualization
    bounding_box_lineset = o3d.geometry.LineSet.create_from_oriented_bounding_box(oriented_bounding_box)
    return bounding_box_lineset

def vector_length(vector):
    return math.sqrt(sum((val ** 2) for val in vector))

def vector_rotate(vector,ratio):
    vector_out = []
    length = vector_length(vector)
    offset = 2* (0.5 - ratio) * length
    for val in vector:
        val = val + offset
        if val > length:
            val = (2 * length) - val
        elif val < (-1*length):
            val = (-2 * length) - val
        vector_out.append(val)
    return vector_out


def create_img_renderer(img_width=1280,img_height=720,fov=50,background=[0, 0, 0, 0],show_axis=False):
    img_renderer = o3d.visualization.rendering.OffscreenRenderer(img_width, img_height)
    img_renderer_height = img_height
    img_renderer_width = img_width
    # Set Lighting -- doesn't change
    img_renderer.scene.set_lighting(img_renderer.scene.LightingProfile.NO_SHADOWS, (0, 0, 0))
    # Set background color
    img_renderer.scene.set_background(background)
    # Set the camera field of view
    vertical_field_of_view = fov  
    aspect_ratio = img_width / img_height  # azimuth over elevation
    near_plane = 0.1
    far_plane = 50.0
    fov_type = o3d.visualization.rendering.Camera.FovType.Vertical
    img_renderer.scene.camera.set_projection(vertical_field_of_view, aspect_ratio, near_plane, far_plane, fov_type)
    # Show the original coordinate axes for comparison.
    # X is red, Y is green and Z is blue.
    img_renderer.scene.show_axes(show_axis)
    return img_renderer

def create_img_renderer_mtl(base_color = [1.0, 1.0, 1.0, 1.0],shader = "defaultUnlit"): # "defaultLit"
    img_renderer_mtl = o3d.visualization.rendering.MaterialRecord()  # or MaterialRecord(), for later versions of Open3D
    # Define a simple unlit Material.
    # (The base color does not replace the arrows' own colors.)
    img_renderer_mtl.base_color = base_color  # RGBA
    img_renderer_mtl.shader = shader
    return img_renderer_mtl
    
def remove_img_renderer_geometry(img_renderer):
    if img_renderer.scene.has_geometry('model'):
        img_renderer.scene.remove_geometry('model')
    return img_renderer
    
def add_img_renderer_geometry(o3d_pc,img_renderer,img_renderer_mtl):
    img_renderer.scene.add_geometry('model',o3d_pc, img_renderer_mtl)
    return img_renderer
     
def render_img(img_renderer,center,eye,up,post_proccessing=False):
    # Look at the origin from the front (along the -Z direction, into the screen), with Y as Up.
    img_renderer.scene.camera.look_at(center, eye, up)
    # Render
    img_renderer.scene.view.set_post_processing(post_proccessing)
    img_o3d = img_renderer.render_to_image()
    return img_o3d
    	

###########################################
### Pointcloud saving functions

def write_pointcloud_file(o3d_pc,filename):
    ''' Save a Open3D PointCloud to a File
        Args:
        o3d_pc (o3d.geometry.PointCloud): Open3D PointCloud
        filename (str): The path to the file where the Open3D PointCloud will be saved
        Returns:
        ret (bool): If true, point cloud was successfully saved
    '''
    ret = o3d.io.write_point_cloud(filename, o3d_pc)
    return ret

def write_pointcloud_image_file(open3d_image,filename):
    ''' Save a Open3D PointCloud Image to a File
        Args:
        open3d_image: Image to be Saved
        filename (str): The path to the file where the image will be saved
        Returns:
        ret (bool): If true, image was successfully saved
    '''
    ret = o3d.io.write_image(filename, open3d_image)
    return ret
    

def read_pointcloud_file(file_path):
    data_from_file = dict()
    if os.path.exists(file_path):
        try:
            data_from_file = o3d.io.read_point_cloud(file_path)
            if data_from_file is not None:
                success = True
        except:
            nepi_msg.publishMsgWarn(self,"Failed to get dict from file: " + file_path + " " + str(e))
    else:
        nepi_msg.publishMsgWarn(self,"Failed to find dict file: " + file_path)
    return data_from_file

def write_o3dpc_2_pcd(data_2_save,file_path):
    success = False
    path = os.path.dirname(file_path)
    if os.path.exists(path):
        try:
            success = nepi_pc.save_pointcloud(data_2_save,file_path)
        except:
            nepi_msg.publishMsgWarn(self,"Failed to write dict: " + str(data_2_save) + " to file: " + file_path + " " + str(e))
    else:
        nepi_msg.publishMsgWarn(self,"Failed to find file path: " + path)
    return success




#### 
# Misc Functions

def get_pc_depth_map_topic(pc_topic):
  topic = pc_topic.rsplit('/',1)[0] + "/depth_map"
  topic = nepi_sdk.find_topic(topic)
  if topic == "":
    topic = None
  return topic
      

def get_pc_pointcloud_img_topic(pc_topic):
  topic = pc_topic.rsplit('/',1)[0] + "/pointcloud_image"
  topic = nepi_sdk.find_topic(topic)
  if topic == "":
    topic = None
  return topic


