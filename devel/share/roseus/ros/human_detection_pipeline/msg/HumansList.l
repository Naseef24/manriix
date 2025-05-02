;; Auto-generated. Do not edit!


(when (boundp 'human_detection_pipeline::HumansList)
  (if (not (find-package "HUMAN_DETECTION_PIPELINE"))
    (make-package "HUMAN_DETECTION_PIPELINE"))
  (shadow 'HumansList (find-package "HUMAN_DETECTION_PIPELINE")))
(unless (find-package "HUMAN_DETECTION_PIPELINE::HUMANSLIST")
  (make-package "HUMAN_DETECTION_PIPELINE::HUMANSLIST"))

(in-package "ROS")
;;//! \htmlinclude HumansList.msg.html


(defclass human_detection_pipeline::HumansList
  :super ros::object
  :slots (_humans ))

(defmethod human_detection_pipeline::HumansList
  (:init
   (&key
    ((:humans __humans) ())
    )
   (send-super :init)
   (setq _humans __humans)
   self)
  (:humans
   (&rest __humans)
   (if (keywordp (car __humans))
       (send* _humans __humans)
     (progn
       (if __humans (setq _humans (car __humans)))
       _humans)))
  (:serialization-length
   ()
   (+
    ;; human_detection_pipeline/HumanTrackingData[] _humans
    (apply #'+ (send-all _humans :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; human_detection_pipeline/HumanTrackingData[] _humans
     (write-long (length _humans) s)
     (dolist (elem _humans)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; human_detection_pipeline/HumanTrackingData[] _humans
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _humans (let (r) (dotimes (i n) (push (instance human_detection_pipeline::HumanTrackingData :init) r)) r))
     (dolist (elem- _humans)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get human_detection_pipeline::HumansList :md5sum-) "c7644a1a2834d6a42a244e016b847ece")
(setf (get human_detection_pipeline::HumansList :datatype-) "human_detection_pipeline/HumansList")
(setf (get human_detection_pipeline::HumansList :definition-)
      "# List of human tracking data
HumanTrackingData[] humans

================================================================================
MSG: human_detection_pipeline/HumanTrackingData
# Define a message for storing the position (x, y, z)
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



(provide :human_detection_pipeline/HumansList "c7644a1a2834d6a42a244e016b847ece")


