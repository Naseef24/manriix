;; Auto-generated. Do not edit!


(when (boundp 'human_clustering_pipeline::OptimalPosition)
  (if (not (find-package "HUMAN_CLUSTERING_PIPELINE"))
    (make-package "HUMAN_CLUSTERING_PIPELINE"))
  (shadow 'OptimalPosition (find-package "HUMAN_CLUSTERING_PIPELINE")))
(unless (find-package "HUMAN_CLUSTERING_PIPELINE::OPTIMALPOSITION")
  (make-package "HUMAN_CLUSTERING_PIPELINE::OPTIMALPOSITION"))

(in-package "ROS")
;;//! \htmlinclude OptimalPosition.msg.html


(defclass human_clustering_pipeline::OptimalPosition
  :super ros::object
  :slots (_x _y _orientation _priority_score ))

(defmethod human_clustering_pipeline::OptimalPosition
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:orientation __orientation) 0.0)
    ((:priority_score __priority_score) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _orientation (float __orientation))
   (setq _priority_score (float __priority_score))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:orientation
   (&optional __orientation)
   (if __orientation (setq _orientation __orientation)) _orientation)
  (:priority_score
   (&optional __priority_score)
   (if __priority_score (setq _priority_score __priority_score)) _priority_score)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _orientation
    8
    ;; float64 _priority_score
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _orientation
       (sys::poke _orientation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _priority_score
       (sys::poke _priority_score (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _orientation
     (setq _orientation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _priority_score
     (setq _priority_score (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get human_clustering_pipeline::OptimalPosition :md5sum-) "8dfd8a1c68c8f6ee8f557969f62c7f98")
(setf (get human_clustering_pipeline::OptimalPosition :datatype-) "human_clustering_pipeline/OptimalPosition")
(setf (get human_clustering_pipeline::OptimalPosition :definition-)
      "float64 x
float64 y
float64 orientation
float64 priority_score
")



(provide :human_clustering_pipeline/OptimalPosition "8dfd8a1c68c8f6ee8f557969f62c7f98")


