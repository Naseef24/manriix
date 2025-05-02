;; Auto-generated. Do not edit!


(when (boundp 'human_detection_pipeline::HumanTrackingData)
  (if (not (find-package "HUMAN_DETECTION_PIPELINE"))
    (make-package "HUMAN_DETECTION_PIPELINE"))
  (shadow 'HumanTrackingData (find-package "HUMAN_DETECTION_PIPELINE")))
(unless (find-package "HUMAN_DETECTION_PIPELINE::HUMANTRACKINGDATA")
  (make-package "HUMAN_DETECTION_PIPELINE::HUMANTRACKINGDATA"))

(in-package "ROS")
;;//! \htmlinclude HumanTrackingData.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass human_detection_pipeline::HumanTrackingData
  :super ros::object
  :slots (_position _tracking_id _distance _confidence ))

(defmethod human_detection_pipeline::HumanTrackingData
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:tracking_id __tracking_id) 0)
    ((:distance __distance) 0.0)
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _position __position)
   (setq _tracking_id (round __tracking_id))
   (setq _distance (float __distance))
   (setq _confidence (float __confidence))
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:tracking_id
   (&optional __tracking_id)
   (if __tracking_id (setq _tracking_id __tracking_id)) _tracking_id)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; int32 _tracking_id
    4
    ;; float32 _distance
    4
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; int32 _tracking_id
       (write-long _tracking_id s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; int32 _tracking_id
     (setq _tracking_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get human_detection_pipeline::HumanTrackingData :md5sum-) "dc13d57981a0f76b7a28dbe20a53f4bb")
(setf (get human_detection_pipeline::HumanTrackingData :datatype-) "human_detection_pipeline/HumanTrackingData")
(setf (get human_detection_pipeline::HumanTrackingData :definition-)
      "# Define a message for storing the position (x, y, z)
geometry_msgs/Point position   # A standard ROS message type for a 3D point (x, y, z)

# Tracking information for one human
int32 tracking_id               # Unique identifier for the human
float32 distance                # Distance to the human
float32 confidence              # Confidence of the tracking data

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :human_detection_pipeline/HumanTrackingData "dc13d57981a0f76b7a28dbe20a53f4bb")


