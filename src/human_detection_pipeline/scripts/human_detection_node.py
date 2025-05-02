#!/usr/bin/env python3

import rospy
import cv2 as cv
import yaml
import os
import numpy as np
import threading
from queue import Queue
from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sensor_msgs.msg 
from concurrent.futures import ThreadPoolExecutor

import sys
# sys.path.append("/home/nsf24/inventa_ws/src/human_detection_pipeline")
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# src/human_detection_pipeline/scripts/human_detection_node.py
from src.detector_copy import HumanDetector
from src.depth_utils import DepthUtils
from src.visualization import Visualizer

#####################33
# from human_detection_pipeline.msg import HumanPosition
from human_detection_pipeline.msg import HumansList, HumanTrackingData
from geometry_msgs.msg import Point

class HumanDetectionNode:
    def __init__(self):
        """Initialize the Human detection node"""
        rospy.init_node("human_detection_node")
        rospy.loginfo("Initializing Human Detection Node...")

        # Load configs
        rospy.loginfo("Loading configuration...")
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        def_config = os.path.join(package_path, "config/config.yaml")
        rospy.loginfo(f"Config path: {def_config}")

        try:
            with open(def_config, 'r') as f:
                self.config = yaml.safe_load(f)
            rospy.loginfo("Configuration loaded successfully")
        except Exception as e:
            rospy.logerr(f"Error loading config: {e}")
            raise
        
        # Initialize components
        rospy.loginfo("Initializing pipeline components...")
        try:
            self.bridge = CvBridge()
            rospy.loginfo("Initializing Human Detector...")
            self.detector = HumanDetector(self.config)
            rospy.loginfo("Initializing Depth Utils...")
            self.depth_utils = DepthUtils(self.config)
            rospy.loginfo("Initializing Visualizer...")
            self.visualizer = Visualizer(self.config)
        except Exception as e:
            rospy.logerr(f"Failed to initialize components: {e}")
            raise

        # Threading setup
        rospy.loginfo("Setting up threading components...")
        self.frame_queue = Queue(maxsize=2)  #Reduced queue size
        self.result_queue = Queue(maxsize=2)
        self.processing_thread = None
        self.visualization_thread = None
        self.is_running = False
        self.lock = threading.Lock()

        # Frame synchronization
        self.latest_rgb = None
        self.latest_depth = None
        self.rgb_timestamp = None
        self.depth_timestamp = None
        self.max_sync_delay = rospy.Duration(0.1)

        #Performance monitoring
        self.frame_times = []
        self.fps = 0
        self.last_fps_update = datetime.now() 

        #Thread pool for parallel processing
        self.thread_pool = ThreadPoolExecutor(max_workers=2)
        

        # Wait for camera intrinsics
        rospy.loginfo("Waiting for camera intrinsics...")
        while not self.depth_utils.intrinsics and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Camera intrinsics received")

        # Initialize subscribers
        rospy.loginfo("Setting up ROS subscribers...")
        try:

            self.rgb_sub = rospy.Subscriber(
                "/camera/color/image_raw",
                Image,
                self.rgb_callback,
                queue_size=1
            )
            # self.depth_sub = rospy.Subscriber(
            #     "/camera/aligned_depth_to_color/image_raw",
            #     Image,
            #     self.depth_callback,
            #     queue_size=1
            # )
            self.depth_sub = rospy.Subscriber(
                "/camera/depth/image_raw",
                Image,
                self.depth_callback,
                queue_size=1
            )            
            rospy.loginfo("ROS subscribers initialized successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize ROS subscribers: {e}")
            raise

        #Publisher
        try:
            rospy.loginfo("Setting up ROS Publisher...")
            self.position_pub = rospy.Publisher(
                'human_detection/human_position',
                HumansList,
                queue_size=10
            )
        except Exception as e:
            rospy.logerr(f"Failed to publish data : {str(e)}")
        
        ######################################

        # Start threads
        rospy.loginfo("Starting processing threads...")
        self.start_threads()
        rospy.loginfo("Initialization complete!")

    def start_threads(self):
        """Start processing threads"""
        try:
            self.is_running = True 
            self.processing_thread = threading.Thread(target=self.processing_loop)
            self.visualization_thread = threading.Thread(target=self.visualization_loop)
            self.processing_thread.start()
            self.visualization_thread.start()
            rospy.loginfo("Processing threads started successfully")
        except Exception as e:
            rospy.logerr(f"Failed to start processing threads: {e}")
            raise

    def stop_threads(self):
        """Stop processing threads"""
        rospy.loginfo("Stopping processing threads...")
        try:
            self.is_running = False 
            if self.processing_thread:
                self.processing_thread.join()
            if self.visualization_thread:
                self.visualization_thread.join()
            self.thread_pool.shutdown()
            rospy.loginfo("Processing threads stopped successfully")
        except Exception as e:
            rospy.logerr(f"Error stopping threads: {e}")

    def rgb_callback(self, msg):
        """Handle incoming rgb frames"""
        try:
            rospy.logdebug("Received RGB frame")
            timestamp = msg.header.stamp
            rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            with self.lock:
                self.latest_rgb = rgb_frame
                self.rgb_timestamp = timestamp
            
            self.try_enqueue_frames()

        except Exception as e:
            rospy.logerr(f"Error in RGB callback: {e}")

    def depth_callback(self, msg):
        """Handle incoming depth frames"""
        try:
            rospy.logdebug("Received depth frame")
            timestamp = msg.header.stamp
            depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            
            with self.lock:
                self.latest_depth = depth_frame
                self.depth_timestamp = timestamp
            
            self.try_enqueue_frames()
            
        except Exception as e:
            rospy.logerr(f"Error in depth callback: {e}")

    def try_enqueue_frames(self):
        """Try to enqueue synchronized frames"""
        with self.lock:
            if(self.latest_rgb is None or self.latest_depth is None or 
               self.rgb_timestamp is None or self.depth_timestamp is None):
                rospy.logdebug("Skipping frame enqueue - missing data")
                return
            
            time_diff = abs(self.rgb_timestamp - self.depth_timestamp)
            if time_diff > self.max_sync_delay:
                rospy.logdebug(f"Frame sync too large: {time_diff.to_sec():.3f}s")
                return

            try:
                if not self.frame_queue.full():
                    self.frame_queue.put({
                        'rgb': self.latest_rgb.copy(),
                        'depth': self.latest_depth.copy(),
                        'timestamp': self.rgb_timestamp
                    })
            except Exception as e:
                rospy.logdebug(f"Queue management error: {e}")

    # ########################Publish method

    def publish_detection(self, detections):
        """Publish detection results"""
        try:
            # Create a HumansList message to hold all detections
            msg = HumansList()

            # Process each detection and create the corresponding HumanTrackingData
            for det in detections:
                if 'position' in det and 'distance' in det:
                    # Create a HumanTrackingData message for this detection
                    human_data = HumanTrackingData()
                    human_data.tracking_id = det["id"]
                    human_data.position = Point(
                        x=det['position']['x'], 
                        y=det['position']['y'], 
                        z=det['position']['z']
                    )
                    human_data.distance = det["distance"]
                    human_data.confidence = det.get("confidence", 0.0)  # Set a default value for confidence if not provided

                    # Append the individual human data to the humans list
                    msg.humans.append(human_data)

            # Publish the complete message with all human detections
            self.position_pub.publish(msg)
            # rospy.loginfo(f"Published {len(msg.humans)} human(s) detection(s).")

        except Exception as e:
            rospy.logerr(f"Error publishing data: {str(e)}")   

    

    def processing_loop(self):
        """Main processing loop"""
        rospy.loginfo("Processing loop started")
        while self.is_running:
            try:
                if self.frame_queue.empty():
                    rospy.sleep(0.01)
                    continue

                frames = self.frame_queue.get()
                rgb_frame = frames['rgb']
                depth_frame = frames['depth']

                # Start timing for FPS calculation
                start_time = datetime.now()
                
                # Scale frames before detection for better performance
                scale_factor = 0.5
                width = int(rgb_frame.shape[1] * scale_factor)
                height = int(rgb_frame.shape[0] * scale_factor)
                rgb_resized = cv.resize(rgb_frame, (width, height))

                # Detect & track humans 
                rospy.logdebug("Running detection and tracking")
                detections = self.detector.detect_and_track(rgb_resized)
                
                # print(detections)



                # Scale back the bounding boxes
                if detections:
                    for det in detections:
                        bbox = det['bbox']
                        det['bbox'] = [
                            bbox[0] / scale_factor,
                            bbox[1] / scale_factor,
                            bbox[2] / scale_factor,
                            bbox[3] / scale_factor
                        ]

                    # Process depth info
                    # rospy.loginfo(f"Detected {len(detections)} humans")
                    detections = self.depth_utils.process_detections(depth_frame, detections)
                    #print(detections)
                    #publish 
                    self.publish_detection(detections)

            
                 # Update FPS
                processing_time = (datetime.now() - start_time).total_seconds()
                self.update_fps(processing_time)

                if not self.result_queue.full():
                    self.result_queue.put({
                        'rgb': rgb_frame,
                        'depth': depth_frame,
                        'detections': detections if detections else [],
                        'fps':self.fps
                    })
                else:
                    rospy.logwarn("Result queue is full, dropping results")

            except Exception as e:
                rospy.logerr(f"Error in processing loop: {e}")

    def visualization_loop(self):
        """Visualization loop"""
        rospy.loginfo("Visualization loop started")
        while self.is_running:
            try:
                if self.result_queue.empty():
                    rospy.sleep(0.01)
                    continue

                # Get processing results
                result = self.result_queue.get()
                rgb_frame = result['rgb']
                depth_frame = result['depth']
                detections = result['detections']
                fps = result.get('fps', 0)

                # Create visualization
                if self.config['visualization']['show_display']:
                    display_frame = self.visualizer.create_display(
                        rgb_frame, depth_frame, detections)

                    # Add FPS
                    cv.putText(display_frame, f"FPS: {self.fps:.1f}",
                                (10, 60), cv.FONT_HERSHEY_SIMPLEX,
                                1, (0, 255, 0), 2)
                    
                    # Show window
                    cv.imshow("Human Detection", display_frame)
                    key = cv.waitKey(1)
                    if key == ord('q'):
                        rospy.loginfo("User requested shutdown")
                        rospy.signal_shutdown("User requested shutdown")
                
                #Print tracking results
                # if detections:
                #     print(self.visualizer.format_detections_json(detections))
                
            except Exception as e:
                rospy.logerr(f"Error in visualization loop: {e}")

   
    def update_fps(self, processing_time):
        """Update FPS calculation"""
        self.frame_times.append(processing_time)
        if len(self.frame_times) > 30:
            self.frame_times.pop(0)
        
        current_time = datetime.now()
        if (current_time - self.last_fps_update).total_seconds() > 1.0:
            if self.frame_times:
                self.fps = 1.0 / np.mean(self.frame_times)
                rospy.logdebug(f"FPS updated: {self.fps:.1f}")
            self.last_fps_update = current_time

    def run(self):
        """Main loop"""
        rospy.loginfo("Node is running and waiting for frames...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutdown requested by user")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean"""
        rospy.loginfo("Performing cleanup...")
        self.stop_threads()
        cv.destroyAllWindows()
        rospy.loginfo("Cleanup complete")

if __name__ == '__main__':
    try:
        node = HumanDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logerr("Node interrupted!")
        pass