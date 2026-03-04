#!/usr/bin/env python3
"""
ZED X ZeroMQ Bridge Node

Subscribes to ROS 2 compressed image topics from 3 ZED X cameras and republishes
them via ZeroMQ IPC for consumption by the AI photo capture system.

Topics subscribed:
    RGB:
        /zedx_front/zed_node/left/image_rect_color/compressed
        /zedx_left/zed_node/left/image_rect_color/compressed
        /zedx_right/zed_node/left/image_rect_color/compressed
    Depth:
        /zedx_front/zed_node/depth/depth_registered/compressedDepth
        /zedx_left/zed_node/depth/depth_registered/compressedDepth
        /zedx_right/zed_node/depth/depth_registered/compressedDepth
    Camera Info:
        /zedx_front/zed_node/left/camera_info
        /zedx_left/zed_node/left/camera_info
        /zedx_right/zed_node/left/camera_info

ZeroMQ publishes:
    Topic: b"zedx_front", b"zedx_left", b"zedx_right"
    Format: msgpack {camera_id, timestamp, format, data, depth_data, depth_format, depth_scale, camera_matrix}

File: zed_zmq_bridge_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
import zmq
import msgpack
import time
from collections import deque


class ZedZmqBridge(Node):

    def __init__(self):
        super().__init__('zed_zmq_bridge')

        # Declare parameters
        self.declare_parameter('ipc_path', '/tmp/zedx_images.ipc')
        self.declare_parameter('cameras', ['front', 'left', 'right'])
        self.declare_parameter('queue_size', 100)  # Increased to 100 for 30Hz × 3 cameras
        self.declare_parameter('publish_stats', True)
        self.declare_parameter('stats_interval', 5.0)
        self.declare_parameter('sync_timeout', 0.05)  # 50ms timeout for RGB+depth sync
        self.declare_parameter('buffer_size', 10)  # Max buffered messages per camera

        # Get parameters
        ipc_path = self.get_parameter('ipc_path').value
        cameras = self.get_parameter('cameras').value
        queue_size = self.get_parameter('queue_size').value
        self.publish_stats = self.get_parameter('publish_stats').value
        self.stats_interval = self.get_parameter('stats_interval').value
        self.sync_timeout = self.get_parameter('sync_timeout').value
        self.buffer_size = self.get_parameter('buffer_size').value

        # QoS - match the direct_zed_detection_node publisher (RELIABLE)
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ZeroMQ setup
        self.get_logger().info('Initializing ZeroMQ publisher...')
        self.zmq_context = zmq.Context()
        self.zmq_pub = self.zmq_context.socket(zmq.PUB)

        zmq_address = f"ipc://{ipc_path}"
        self.zmq_pub.bind(zmq_address)
        self.zmq_pub.setsockopt(zmq.SNDHWM, queue_size)
        self.zmq_pub.setsockopt(zmq.LINGER, 0)

        self.get_logger().info(f'ZeroMQ publisher bound to: {zmq_address}')

        # Data buffers for synchronization (RGB, depth, camera_info)
        self.rgb_buffers = {cam: deque(maxlen=self.buffer_size) for cam in cameras}
        self.depth_buffers = {cam: deque(maxlen=self.buffer_size) for cam in cameras}
        self.camera_info = {cam: None for cam in cameras}

        # Statistics
        self.stats = {cam: {'count': 0, 'bytes': 0, 'last_time': 0, 'depth_count': 0, 'synced_count': 0, 'dropped': 0}
                      for cam in cameras}
        self.last_stats_time = time.time()

        # Subscribe to camera topics (matching direct_zed_detection_node)
        self.camera_subs = []
        for camera in cameras:
            # RGB topic
            rgb_topic = f'/zedx_{camera}/zed_node/left/image_rect_color/compressed'
            rgb_sub = self.create_subscription(
                CompressedImage,
                rgb_topic,
                lambda msg, cam=camera: self.rgb_callback(msg, cam),
                self.image_qos)
            self.camera_subs.append(rgb_sub)
            self.get_logger().info(f'Subscribed to RGB: {rgb_topic}')

            # Depth topic
            depth_topic = f'/zedx_{camera}/zed_node/depth/depth_registered/compressedDepth'
            depth_sub = self.create_subscription(
                CompressedImage,
                depth_topic,
                lambda msg, cam=camera: self.depth_callback(msg, cam),
                self.image_qos)
            self.camera_subs.append(depth_sub)
            self.get_logger().info(f'Subscribed to Depth: {depth_topic}')

            # Camera info topic
            info_topic = f'/zedx_{camera}/zed_node/left/camera_info'
            info_sub = self.create_subscription(
                CameraInfo,
                info_topic,
                lambda msg, cam=camera: self.camera_info_callback(msg, cam),
                self.image_qos)
            self.camera_subs.append(info_sub)
            self.get_logger().info(f'Subscribed to Camera Info: {info_topic}')

        # Statistics timer
        if self.publish_stats:
            self.stats_timer = self.create_timer(
                self.stats_interval,
                self.print_statistics)

        self.get_logger().info(
            f'ZED ZMQ Bridge initialized for {len(cameras)} cameras')

    def rgb_callback(self, msg: CompressedImage, camera_id: str):
        """Buffer RGB image and attempt to publish synchronized frame"""
        try:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # Store in buffer
            self.rgb_buffers[camera_id].append({
                'timestamp': timestamp,
                'format': msg.format,
                'data': bytes(msg.data)
            })

            # Update stats
            self.stats[camera_id]['count'] += 1
            self.stats[camera_id]['bytes'] += len(msg.data)
            self.stats[camera_id]['last_time'] = timestamp

            # Try to publish synchronized frame
            self.try_publish_synchronized(camera_id)

        except Exception as e:
            self.get_logger().error(
                f'Error in rgb_callback for {camera_id}: {e}',
                throttle_duration_sec=1.0)

    def depth_callback(self, msg: CompressedImage, camera_id: str):
        """Buffer depth image and attempt to publish synchronized frame"""
        try:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # Store in buffer
            self.depth_buffers[camera_id].append({
                'timestamp': timestamp,
                'format': msg.format,
                'data': bytes(msg.data)
            })

            # Update stats
            self.stats[camera_id]['depth_count'] += 1

            # Try to publish synchronized frame
            self.try_publish_synchronized(camera_id)

        except Exception as e:
            self.get_logger().error(
                f'Error in depth_callback for {camera_id}: {e}',
                throttle_duration_sec=1.0)

    def camera_info_callback(self, msg: CameraInfo, camera_id: str):
        """Store camera info (only needs to be received once)"""
        try:
            # Extract intrinsic matrix
            K = msg.k  # 3x3 matrix stored as flat array [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            
            self.camera_info[camera_id] = {
                'fx': K[0],
                'fy': K[4],
                'cx': K[2],
                'cy': K[5],
                'width': msg.width,
                'height': msg.height
            }

            self.get_logger().info(
                f'Camera info received for {camera_id}: '
                f'{msg.width}x{msg.height}, fx={K[0]:.1f}, fy={K[4]:.1f}',
                throttle_duration_sec=10.0)

        except Exception as e:
            self.get_logger().error(
                f'Error in camera_info_callback for {camera_id}: {e}',
                throttle_duration_sec=1.0)

    def try_publish_synchronized(self, camera_id: str):
        """Try to find and publish a synchronized RGB+depth pair"""
        try:
            # Need RGB buffer, depth buffer, and camera info
            if not self.rgb_buffers[camera_id] or not self.depth_buffers[camera_id]:
                return

            if self.camera_info[camera_id] is None:
                # Camera info not yet received, can still publish RGB-only for backward compatibility
                pass

            # Find best matching RGB and depth frames
            rgb_data = None
            depth_data = None
            min_time_diff = float('inf')

            # Try each RGB frame
            for rgb in self.rgb_buffers[camera_id]:
                # Find closest depth frame
                for depth in self.depth_buffers[camera_id]:
                    time_diff = abs(rgb['timestamp'] - depth['timestamp'])
                    
                    if time_diff < min_time_diff and time_diff <= self.sync_timeout:
                        min_time_diff = time_diff
                        rgb_data = rgb
                        depth_data = depth

            # Publish if we found a synchronized pair
            if rgb_data is not None and depth_data is not None:
                self.publish_frame(camera_id, rgb_data, depth_data, min_time_diff)
                
                # Remove old frames from buffers (keep only newer than published frame)
                self.clean_buffers(camera_id, rgb_data['timestamp'])

        except Exception as e:
            self.get_logger().error(
                f'Error in try_publish_synchronized for {camera_id}: {e}',
                throttle_duration_sec=1.0)

    def publish_frame(self, camera_id: str, rgb_data: dict, depth_data: dict, time_diff: float):
        """Publish synchronized RGB+depth frame via ZeroMQ"""
        try:
            # Build payload with all data
            payload_data = {
                'camera_id': f'zedx_{camera_id}',
                'timestamp': rgb_data['timestamp'],
                'format': rgb_data['format'],
                'data': rgb_data['data'],
                'depth_format': depth_data['format'],
                'depth_data': depth_data['data'],
                'depth_scale': 0.001,  # ZED depth is in mm, scale to meters
            }

            # Add camera matrix if available
            if self.camera_info[camera_id] is not None:
                payload_data['camera_matrix'] = self.camera_info[camera_id]

            # CRITICAL FIX: Use single-part message instead of multipart
            # Embed topic in payload to avoid multipart fragmentation issues
            topic_name = f'zedx_{camera_id}'
            combined_payload = {
                'topic': topic_name,
                'data': payload_data
            }
            
            # Pack combined message
            message = msgpack.packb(combined_payload, use_bin_type=True)

            # Validate message before sending
            if not isinstance(message, bytes) or len(message) == 0:
                self.get_logger().error(f'Invalid message for {camera_id}')
                return

            # Send as SINGLE-PART message (prevents fragmentation)
            # Using NOBLOCK to prevent blocking, but we track drops
            try:
                self.zmq_pub.send(message, flags=zmq.NOBLOCK, copy=False)
                
                # Update stats only if send succeeded
                self.stats[camera_id]['synced_count'] += 1
                
            except zmq.Again:
                # Queue full - track drops
                if 'dropped' not in self.stats[camera_id]:
                    self.stats[camera_id]['dropped'] = 0
                self.stats[camera_id]['dropped'] += 1
                
                self.get_logger().warning(
                    f'ZMQ queue full for {camera_id}, dropping frame (total drops: {self.stats[camera_id]["dropped"]})',
                    throttle_duration_sec=2.0)

        except Exception as e:
            self.get_logger().error(
                f'Error in publish_frame for {camera_id}: {e}',
                throttle_duration_sec=1.0)

    def clean_buffers(self, camera_id: str, published_timestamp: float):
        """Remove frames older than the published timestamp from buffers"""
        try:
            # Clean RGB buffer
            while self.rgb_buffers[camera_id] and \
                  self.rgb_buffers[camera_id][0]['timestamp'] <= published_timestamp:
                self.rgb_buffers[camera_id].popleft()

            # Clean depth buffer
            while self.depth_buffers[camera_id] and \
                  self.depth_buffers[camera_id][0]['timestamp'] <= published_timestamp:
                self.depth_buffers[camera_id].popleft()

        except Exception as e:
            self.get_logger().error(
                f'Error cleaning buffers for {camera_id}: {e}',
                throttle_duration_sec=1.0)

    def image_callback(self, msg: CompressedImage, camera_id: str):
        """Legacy callback - deprecated, keeping for backward compatibility"""
        self.get_logger().warning(
            'image_callback is deprecated, use rgb_callback instead',
            throttle_duration_sec=10.0)
        self.rgb_callback(msg, camera_id)

    def print_statistics(self):
        """Print forwarding rate statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_stats_time

        if elapsed < 0.1:
            return

        parts = []
        for camera_id, stats in self.stats.items():
            if stats['count'] > 0 or stats['synced_count'] > 0:
                rgb_fps = stats['count'] / elapsed
                depth_fps = stats['depth_count'] / elapsed
                synced_fps = stats['synced_count'] / elapsed
                dropped = stats.get('dropped', 0)
                
                parts.append(
                    f"{camera_id}: RGB={rgb_fps:.1f}Hz, "
                    f"Depth={depth_fps:.1f}Hz, Synced={synced_fps:.1f}Hz, "
                    f"Dropped={dropped}"
                )
                
                # Reset counters
                stats['count'] = 0
                stats['bytes'] = 0
                stats['depth_count'] = 0
                stats['synced_count'] = 0
                stats['dropped'] = 0

        if parts:
            self.get_logger().info(f"ZMQ Bridge Stats:\n  " + "\n  ".join(parts))

        self.last_stats_time = current_time

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down ZED ZMQ Bridge...')
        if hasattr(self, 'zmq_pub'):
            self.zmq_pub.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedZmqBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()