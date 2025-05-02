#!/usr/bin/env python3

import os
from ultralytics import YOLO
import rospy
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
        
        # Load or download model
        self.model = self.load_or_download_model()

    def load_or_download_model(self):
        """Load or Download the model"""
        try:
            if not os.path.exists(self.model_path):
                print(f"Model not found at {self.model_path}")
                print(f" ⌛ Downloading model {self.model_name}...")
                
                # Download model
                model = YOLO(self.model_name)
                
                # Save model to specified directory
                print(f"Saving model to {self.model_path}")
                model.save(self.model_path)
                print(" ✅ Model downloaded and saved successfully!")
            else:
                print(f" Loading existing model from {self.model_path}")
            
            # Load model and move to specified device
            model = YOLO(self.model_path)
            model.to(self.device)
            print(f"Model loaded successfully on {self.device}!")
            
            return model
            
        except Exception as e:
            print(f"Error in model loading/downloading: {e}")
            raise
 
        
    def detect_and_track(self, frame):
        """Detect Humans
            args : RGB frame(np.ndarray)
            returns: list with bbox and CI
        """
        try:
            results = self.model.track(frame,
                                       conf= self.confidence_threshold,
                                       verbose= False)
            detections = []

            if results and len(results) > 0:
                boxes = results[0].boxes
                if boxes is not None and len(boxes) > 0:
                    for box in boxes:
                        if box.cls.cpu().numpy()[0] == 0:  #class 0 -> Person
                            track_id = -1
                            if hasattr(box, 'id') and box.id is not None:
                                track_id = int(box.id.cpu().numpy()[0])
                            
                            detection = {
                                "id": track_id,
                                "bbox": box.xyxy.cpu().numpy()[0],  # [x1, y1, x2, y2]
                                "confidence": float(box.conf.cpu().numpy()[0])
                            }
                            detections.append(detection)
            return detections
        
        except Exception as e:
            print(f"Error in Detection and Tracking : {e}")
            return []