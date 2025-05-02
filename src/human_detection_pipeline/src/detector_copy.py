#!/usr/bin/env python3

import os
from ultralytics import YOLO
import rospy
import supervision as sv

class HumanDetector:
    def __init__(self, config):
        """Initialize the detector -> YOLO
           args: config(dict)
        """
        self.config = config 
        self.model_name = config['model']['name']
        
        self.confidence_threshold = config['model']['confidence_threshold']
        self.device = config['model']['device']
        self.model_dir = config['model']['model_dir']

        os.makedirs(self.model_dir, exist_ok=True)

        self.model_path = os.path.join(self.model_dir, self.model_name)
        
        # detection model
        self.model = self.load_or_download_model()

        #tracker
        self.tracker = sv.ByteTrack()

    def load_or_download_model(self):
        """Load or Download the model"""
        try:
            if not os.path.exists(self.model_path):
                rospy.logerr(f"Model not found at {self.model_path}")
                rospy.loginfo(f" ⌛ Downloading model {self.model_name}...")
                
                # Download model
                model = YOLO(self.model_name)
                
                # Save model to specified directory
                rospy.loginfo(f"Saving model to {self.model_path}")
                model.save(self.model_path)
                rospy.loginfo(" ✅ Model downloaded and saved successfully!")
            else:
                rospy.loginfo(f" Loading existing model from {self.model_path}")
            
            # Load model and move to specified device
            model = YOLO(self.model_path)
            model.to(self.device)
            rospy.loginfo(f"Model loaded successfully on {self.device}!")
            
            return model
            
        except Exception as e:
            rospy.loginfo(f"Error in model loading/downloading: {e}")
            raise
 
        
    def detect_and_track(self, frame):
        """
        Detect --> YOLO  
        Track --> supervision's ByteTrack
        """
        try:
            # detection with YOLO
            results = self.model(
                frame,
                conf=self.confidence_threshold,
                verbose=False
            )

            detections = []
            if results and len(results) > 0:
                result = results[0]
                
                # Convert YOLO detections to supervision format
                boxes = result.boxes.xyxy.cpu().numpy() 
                confidence = result.boxes.conf.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy()
                
                # Create supervision detections
                detections_sv = sv.Detections(
                    xyxy=boxes,
                    confidence=confidence,
                    class_id=class_ids
                )
                
                # get humans (class_id  = 0)
                mask = detections_sv.class_id == 0
                detections_sv = detections_sv[mask]
                
                # Track detections
                tracked_detections = self.tracker.update_with_detections(
                    detections=detections_sv 
                )
                
                # Convert tracked detections to our format
                if tracked_detections is not None and len(tracked_detections) > 0:
                    for i in range(len(tracked_detections)):
                        # Get tracking ID
                        track_id = tracked_detections.tracker_id[i]
                        if track_id is not None: 
                            detection = {
                                "id": int(track_id),
                                "bbox": tracked_detections.xyxy[i],  
                                "confidence": tracked_detections.confidence[i]
                            }
                            detections.append(detection)
                            rospy.logdebug(f"Detected person with ID: {track_id}")

            return detections
            
        except Exception as e:
            rospy.logerr(f"Error in detection and tracking: {e}")
            return []


#!/usr/bin/env python3

# import os
# from ultralytics import YOLO
# import rospy
# import supervision as sv
# import numpy as np
# from collections import deque

# class HumanDetector:
#     def __init__(self, config):
#         """Initialize the detector -> YOLO
#            args: config(dict)
#         """
#         self.config = config 
#         self.model_name = config['model']['name']
        
#         self.confidence_threshold = config['model']['confidence_threshold']
#         self.device = config['model']['device']
#         self.model_dir = config['model']['model_dir']

#         os.makedirs(self.model_dir, exist_ok=True)
#         self.model_path = os.path.join(self.model_dir, self.model_name)
        
#         # detection model
#         self.model = self.load_or_download_model()

#         # Tracker parameters
#         self.track_thresh = config.get('tracker', {}).get('track_thresh', 0.4)
#         self.track_buffer = config.get('tracker', {}).get('track_buffer', 30)
#         self.match_thresh = config.get('tracker', {}).get('match_thresh', 0.3)
#         self.frame_rate = config.get('tracker', {}).get('frame_rate', 30)

#         # Initialize tracker
#         self.tracker = sv.ByteTrack(
#             track_activation_threshold=self.track_thresh,
#             lost_track_buffer=self.track_buffer,
#             minimum_matching_threshold=self.match_thresh,
#             frame_rate=self.frame_rate
#         )
        
#         # Initialize tracking history
#         self.track_history = {}
#         self.max_history = config.get('tracker', {}).get('max_history', 10)
        
#         # Additional parameters for movement consistency
#         self.max_movement_multiplier = config.get('tracker', {}).get('max_movement_multiplier', 3.0)
#         self.min_detection_confidence = config.get('tracker', {}).get('min_detection_confidence', 0.3)

#     def load_or_download_model(self):
#         """Load or Download the model"""
#         try:
#             if not os.path.exists(self.model_path):
#                 rospy.loginfo(f"Model not found at {self.model_path}")
#                 rospy.loginfo(f"⌛ Downloading model {self.model_name}...")
                
#                 # Download model
#                 model = YOLO(self.model_name)
                
#                 # Save model to specified directory
#                 rospy.loginfo(f"Saving model to {self.model_path}")
#                 model.save(self.model_path)
#                 rospy.loginfo("✅ Model downloaded and saved successfully!")
#             else:
#                 rospy.loginfo(f"Loading existing model from {self.model_path}")
            
#             # Load model and move to specified device
#             model = YOLO(self.model_path)
#             model.to(self.device)
#             rospy.loginfo(f"Model loaded successfully on {self.device}!")
            
#             return model
            
#         except Exception as e:
#             rospy.logerr(f"Error in model loading/downloading: {e}")
#             raise

#     def update_track_history(self, track_id, bbox):
#         """Update tracking history with smoother trajectory"""
#         center = ((bbox[0] + bbox[2])/2, (bbox[1] + bbox[3])/2)

#         if track_id not in self.track_history:
#             self.track_history[track_id] = deque(maxlen=self.max_history)
        
#         # Add smoothing for sudden jumps
#         if len(self.track_history[track_id]) > 0:
#             last_center = self.track_history[track_id][-1]
#             smoothed_center = (
#                 0.7 * center[0] + 0.3 * last_center[0],
#                 0.7 * center[1] + 0.3 * last_center[1]
#             )
#             self.track_history[track_id].append(smoothed_center)
#         else:
#             self.track_history[track_id].append(center)

#     def is_track_consistent(self, track_id, current_center):
#         """Check track consistency with adaptive thresholds"""
#         if track_id not in self.track_history or len(self.track_history[track_id]) < 3:
#             return True 
        
#         history = self.track_history[track_id]
#         distances = []

#         for i in range(1, len(history)):
#             dist = np.sqrt((history[i][0] - history[i-1][0])**2 +
#                           (history[i][1] - history[i-1][1])**2)
#             distances.append(dist)

#         avg_movement = np.mean(distances)
#         current_movement = np.sqrt((current_center[0] - history[-1][0])**2 +
#                                  (current_center[1] - history[-1][1])**2)
        
#         # Adaptive threshold based on average movement
#         max_allowed_movement = max(
#             avg_movement * self.max_movement_multiplier,
#             50.0  # Minimum threshold to allow some movement
#         )
        
#         return current_movement <= max_allowed_movement

#     def detect_and_track(self, frame):
#         """
#         Detect --> YOLO  
#         Track --> supervision's ByteTrack with improved consistency
#         """
#         try:
#             # Debug frame information
#             rospy.logdebug(f"Processing frame shape: {frame.shape}")
            
#             # Detection with YOLO
#             results = self.model(
#                 frame,
#                 conf=self.confidence_threshold,
#                 verbose=False
#             )

#             detections = []
#             if results and len(results) > 0:
#                 result = results[0]
                
#                 # Convert YOLO detections to supervision format
#                 boxes = result.boxes.xyxy.cpu().numpy() 
#                 confidence = result.boxes.conf.cpu().numpy()
#                 class_ids = result.boxes.cls.cpu().numpy()
                
#                 # Debug detection counts
#                 rospy.logdebug(f"Raw detections: {len(boxes)}")
                
#                 # Create supervision detections
#                 detections_sv = sv.Detections(
#                     xyxy=boxes,
#                     confidence=confidence,
#                     class_id=class_ids
#                 )
                
#                 # Filter for humans (class_id = 0) and minimum confidence
#                 mask = (detections_sv.class_id == 0) & (detections_sv.confidence >= self.min_detection_confidence)
#                 detections_sv = detections_sv[mask]
                
#                 rospy.logdebug(f"Filtered human detections: {len(detections_sv)}")
                
#                 # Track detections
#                 tracked_detections = self.tracker.update_with_detections(detections_sv)
                
#                 # Process tracked detections
#                 if tracked_detections is not None and len(tracked_detections) > 0:
#                     for i in range(len(tracked_detections)):
#                         track_id = tracked_detections.tracker_id[i]
#                         if track_id is not None:
#                             bbox = tracked_detections.xyxy[i]
#                             center = ((bbox[0] + bbox[2])/2, (bbox[1] + bbox[3])/2)
                            
#                             # Check track consistency
#                             if self.is_track_consistent(int(track_id), center):
#                                 detection = {
#                                     "id": int(track_id),
#                                     "bbox": bbox,
#                                     "confidence": tracked_detections.confidence[i]
#                                 }
#                                 detections.append(detection)
#                                 self.update_track_history(int(track_id), bbox)
#                                 rospy.logdebug(f"Tracked person with ID: {track_id}")
                
#                 rospy.logdebug(f"Final tracked detections: {len(detections)}")

#             return detections
            
#         except Exception as e:
#             rospy.logerr(f"Error in detection and tracking: {e}")
#             return []
        
#     def cleanup(self):
#         """Cleanup tracking history"""
#         self.track_history.clear()