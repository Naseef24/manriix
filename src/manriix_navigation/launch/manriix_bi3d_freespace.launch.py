# =============================================================================
# Manriix Bi3D-Freespace launch — low-obstacle detection via Bi3D DNN
#
# Pipeline (two-stage image preprocessing, matches NVIDIA's pattern):
#   ZED front color (BGRA8 960x600) + ZED camera_info
#     -> ImageFormatConverterNode (BGRA8 -> RGB8, stays 960x600)
#     -> ResizeNode (RGB8 960x600 -> RGB8 960x576, scales camera_info)
#     -> Bi3DNode (DLA core 0, featnet + segnet at 960x576)
#     -> FreespaceSegmentationNode (occupancy grid for Nav2)
#
# Why two stages?  ResizeNode's tensorops kernel does NOT support BGRA8 input.
# It declared its subscriber as nitros_image_rgb8 via encoding_desired but the
# actual incoming bytes were still BGRA8 from ZED, causing:
#   ERROR Resize.cpp@229: invalid input/output type for image resize.
# So we put ImageFormatConverter BEFORE ResizeNode to do the color conversion.
#
# Why Bi3D pool was failing before: ImageFormatConverter does NOT resize. Its
# image_width/height params validate INPUT dims only. So with only that node,
# Bi3D received 960x600 RGB8, allocated [N x 600 x 960 x float] tensors, and
# crashed because the pool was sized for 960x576.
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # ---------- Defaults ----------
    HOME = os.path.expanduser('~')
    DEFAULT_MODEL_DIR = os.path.join(
        HOME, 'manriix2_ws', 'isaac_ros_assets', 'models',
        'bi3d_proximity_segmentation')

    # Native ZED output dimensions
    ZED_W = 960
    ZED_H = 600

    # Bi3D model input dimensions (HARD-CODED in the model)
    BI3D_W = 960
    BI3D_H = 576

    # ZED front intrinsics (verified from /zedx_front/zed_node/left/color/rect/camera_info)
    # ResizeNode auto-scales camera_info, but freespace_segmentation_node takes
    # f_x/f_y as direct PARAMETERS (not from topic), so we pre-scale here.
    ZED_F_X_NATIVE = 368.5843505859375  # for 960 width (unchanged after horizontal resize)
    ZED_F_Y_NATIVE = 368.5843505859375  # for 600 height (needs vertical scaling)
    ZED_F_Y_RESIZED = ZED_F_Y_NATIVE * (BI3D_H / 600.0)  # = 353.840...

    # ---------- Launch arguments ----------
    launch_args = [
        DeclareLaunchArgument(
            'featnet_engine_file_path',
            default_value=os.path.join(DEFAULT_MODEL_DIR, 'featnet.plan'),
            description='Path to Bi3D Featnet TensorRT engine'),
        DeclareLaunchArgument(
            'segnet_engine_file_path',
            default_value=os.path.join(DEFAULT_MODEL_DIR, 'segnet.plan'),
            description='Path to Bi3D Segnet TensorRT engine'),
        DeclareLaunchArgument(
            'max_disparity_values',
            default_value='64',
            description='Maximum number of disparity values for Bi3D inference'),
        DeclareLaunchArgument(
            'base_link_frame',
            default_value='base_footprint',
            description='Robot base frame (Manriix uses base_footprint)'),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='zedx_front_left_camera_frame_optical',
            description='ZED front-left optical frame'),
        DeclareLaunchArgument(
            'grid_height',
            default_value='2000',
            description='Occupancy grid height (cells). 2000 * 0.01 = 20m'),
        DeclareLaunchArgument(
            'grid_width',
            default_value='2000',
            description='Occupancy grid width (cells)'),
        DeclareLaunchArgument(
            'grid_resolution',
            default_value='0.01',
            description='Cell size (m). 0.01 = 1 cm'),
    ]

    featnet_engine_file_path = LaunchConfiguration('featnet_engine_file_path')
    segnet_engine_file_path = LaunchConfiguration('segnet_engine_file_path')
    max_disparity_values = LaunchConfiguration('max_disparity_values')
    base_link_frame = LaunchConfiguration('base_link_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    grid_height = LaunchConfiguration('grid_height')
    grid_width = LaunchConfiguration('grid_width')
    grid_resolution = LaunchConfiguration('grid_resolution')

    # ---------- Stage 1: BGRA8 -> RGB8 (no resize) for LEFT ----------
    format_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='bi3d_format_left',
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': ZED_W,
            'image_height': ZED_H,
        }],
        remappings=[
            ('image_raw', '/zedx_front/zed_node/left/color/rect/image'),
            ('image',     '/bi3d/left/image_rgb'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # ---------- Stage 1: BGRA8 -> RGB8 (no resize) for RIGHT ----------
    format_right = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='bi3d_format_right',
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': ZED_W,
            'image_height': ZED_H,
        }],
        remappings=[
            ('image_raw', '/zedx_front/zed_node/right/color/rect/image'),
            ('image',     '/bi3d/right/image_rgb'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # ---------- Stage 2: RGB8 960x600 -> RGB8 960x576 for LEFT ----------
    resize_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='bi3d_resize_left',
        parameters=[{
            'output_width': BI3D_W,
            'output_height': BI3D_H,
            'encoding_desired': 'rgb8',
            'keep_aspect_ratio': False,
        }],
        remappings=[
            ('image',       '/bi3d/left/image_rgb'),
            ('camera_info', '/zedx_front/zed_node/left/color/rect/camera_info'),
            ('resize/image',       '/bi3d/left/image_resize'),
            ('resize/camera_info', '/bi3d/left/camera_info_resize'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # ---------- Stage 2: RGB8 960x600 -> RGB8 960x576 for RIGHT ----------
    resize_right = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='bi3d_resize_right',
        parameters=[{
            'output_width': BI3D_W,
            'output_height': BI3D_H,
            'encoding_desired': 'rgb8',
            'keep_aspect_ratio': False,
        }],
        remappings=[
            ('image',       '/bi3d/right/image_rgb'),
            ('camera_info', '/zedx_front/zed_node/right/color/rect/camera_info'),
            ('resize/image',       '/bi3d/right/image_resize'),
            ('resize/camera_info', '/bi3d/right/camera_info_resize'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # ---------- Bi3D inference node ----------
    bi3d_node = ComposableNode(
        package='isaac_ros_bi3d',
        plugin='nvidia::isaac_ros::bi3d::Bi3DNode',
        name='bi3d_node',
        parameters=[{
            'image_width': BI3D_W,
            'image_height': BI3D_H,
            'featnet_engine_file_path': featnet_engine_file_path,
            'segnet_engine_file_path': segnet_engine_file_path,
            'max_disparity_values': max_disparity_values,
            'disparity_values': [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 30, 40, 50, 60],
        }],
        remappings=[
            ('left_image_bi3d',       '/bi3d/left/image_resize'),
            ('right_image_bi3d',      '/bi3d/right/image_resize'),
            ('left_camera_info_bi3d', '/bi3d/left/camera_info_resize'),
            ('right_camera_info_bi3d','/bi3d/right/camera_info_resize'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # ---------- Freespace segmentation -> occupancy grid for Nav2 ----------
    freespace_segmentation_node = ComposableNode(
        package='isaac_ros_bi3d_freespace',
        plugin='nvidia::isaac_ros::bi3d_freespace::FreespaceSegmentationNode',
        name='freespace_segmentation_node',
        parameters=[{
            'base_link_frame': base_link_frame,
            'camera_frame': camera_frame,
            'f_x': ZED_F_X_NATIVE,
            'f_y': ZED_F_Y_RESIZED,
            'grid_height': grid_height,
            'grid_width': grid_width,
            'grid_resolution': grid_resolution,
        }],
        remappings=[
            ('bi3d_mask', '/bi3d_node/bi3d_output'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # ---------- Container ----------
    bi3d_container = ComposableNodeContainer(
        name='bi3d_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            format_left,
            format_right,
            resize_left,
            resize_right,
            bi3d_node,
            freespace_segmentation_node,
        ],
        output='screen',
    )

    return LaunchDescription(launch_args + [bi3d_container])
