#!/usr/bin/env python3

import cv2
import numpy as np
import json
import rospy

class Visualizer:
    def __init__(self, config):
        """Initialize visualizer"""
        self.show_display = config['visualization']['show_display']
        self.text_color = tuple(config['visualization']['text_color'])
        self.bbox_color = tuple(config['visualization']['bbox_color'])

    def draw_detection(self, image, detection):
        """Draw detection on image"""
        try:
            bbox = [int(x) for x in detection['bbox']]
            
            # Draw bounding box
            cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), 
                         self.bbox_color, 2)
            
            # Prepare text parts
            text_parts = []
            if 'id' in detection and detection['id'] != -1:
                text_parts.append(f"ID:{detection['id']}")
            if 'confidence' in detection:
                text_parts.append(f"{detection['confidence']:.2f}")
            if 'distance' in detection:
                text_parts.append(f"{detection['distance']:.2f}m")

            

            text = " ".join(text_parts)
            
            # Calculate text size
            (text_width, text_height), baseline = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            
            # if 'position' in detection:
            #     pos = detection['position']
            #     pos_text = f"X:{pos['x']:.2f} Y:{pos['y']:.2f} Z:{pos['z']:.2f}"
                
            #     cv2.putText(image, pos_text,
            #             (bbox[0], bbox[1] - text_height - 10),
            #             cv2.FONT_HERSHEY_SIMPLEX,
            #             0.4, self.text_color, 1)
            # Draw text background
            cv2.rectangle(image,
                         (bbox[0], bbox[1] - text_height - 8),
                         (bbox[0] + text_width, bbox[1]),
                         self.bbox_color, -1)
            
            # Draw text
            cv2.putText(image, text,
                        (bbox[0], bbox[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0), 2)
            
            # add center point
            cv2.circle(image, 
                       (detection["center_point"]["center_x"], detection["center_point"]["center_y"]), 
                       radius=5, 
                       color=(0, 0, 255), 
                       thickness=-1) 
                        
        except Exception as e:
            rospy.logerr(f"Error drawing detection: {e}")

    def draw_human_count(self, image, detections):
        """Draw human count on image"""
        count = len(detections) if detections else 0
        text = f"Humans Detected: {count}"
        cv2.putText(image, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, self.text_color, 2)

    def process_depth_image(self, depth_image):
        """Process depth image for visualization"""
        try:
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, 
                                          cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            return depth_colormap
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")
            return np.zeros_like(depth_image)

    def create_display(self, rgb_image, depth_image, detections):
        """Create complete visualization"""
        try:
            rgb_viz = rgb_image.copy()
            depth_colormap = self.process_depth_image(depth_image)

            # Draw detections
            if detections:
                for det in detections:
                    self.draw_detection(rgb_viz, det)
                    self.draw_detection(depth_colormap, det)

            # Draw human count
            self.draw_human_count(rgb_viz, detections if detections else [])

            # Calculate target size
            target_height = min(480, rgb_viz.shape[0])  
            target_width = int((target_height / rgb_viz.shape[0]) * rgb_viz.shape[1])

            # Resize both images
            rgb_viz_resized = cv2.resize(rgb_viz, (target_width, target_height))
            depth_colormap_resized = cv2.resize(depth_colormap, (target_width, target_height))

            # Combine images horizontally
            combined = np.hstack((rgb_viz_resized, depth_colormap_resized))

            return combined

        except Exception as e:
            rospy.logerr(f"Error in create_display: {e}")
            return rgb_image

    def format_detections_json(self, detections):
        """Format detections as JSON string"""
        try:
            if not detections:
                return json.dumps({"humans": []}, indent=2)
                
            output = {
                "humans": [
                    {
                        "tracking_id": det["id"],
                        "position": det["position"] if "position" in det else None,
                        "distance": float(det["distance"]) if "distance" in det else None,
                        "confidence": float(det["confidence"])
                    } for det in detections
                ]
            }
            
            return json.dumps(output, indent=2)
        except Exception as e:
            rospy.logerr(f"Error formatting detections JSON: {e}")
            return json.dumps({"humans": [], "error": str(e)}, indent=2)