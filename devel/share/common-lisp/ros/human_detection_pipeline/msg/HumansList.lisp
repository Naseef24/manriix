; Auto-generated. Do not edit!


(cl:in-package human_detection_pipeline-msg)


;//! \htmlinclude HumansList.msg.html

(cl:defclass <HumansList> (roslisp-msg-protocol:ros-message)
  ((humans
    :reader humans
    :initarg :humans
    :type (cl:vector human_detection_pipeline-msg:HumanTrackingData)
   :initform (cl:make-array 0 :element-type 'human_detection_pipeline-msg:HumanTrackingData :initial-element (cl:make-instance 'human_detection_pipeline-msg:HumanTrackingData))))
)

(cl:defclass HumansList (<HumansList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumansList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumansList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name human_detection_pipeline-msg:<HumansList> is deprecated: use human_detection_pipeline-msg:HumansList instead.")))

(cl:ensure-generic-function 'humans-val :lambda-list '(m))
(cl:defmethod humans-val ((m <HumansList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_detection_pipeline-msg:humans-val is deprecated.  Use human_detection_pipeline-msg:humans instead.")
  (humans m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumansList>) ostream)
  "Serializes a message object of type '<HumansList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'humans))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'humans))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumansList>) istream)
  "Deserializes a message object of type '<HumansList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'humans) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'humans)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'human_detection_pipeline-msg:HumanTrackingData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumansList>)))
  "Returns string type for a message object of type '<HumansList>"
  "human_detection_pipeline/HumansList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumansList)))
  "Returns string type for a message object of type 'HumansList"
  "human_detection_pipeline/HumansList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumansList>)))
  "Returns md5sum for a message object of type '<HumansList>"
  "c7644a1a2834d6a42a244e016b847ece")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumansList)))
  "Returns md5sum for a message object of type 'HumansList"
  "c7644a1a2834d6a42a244e016b847ece")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumansList>)))
  "Returns full string definition for message of type '<HumansList>"
  (cl:format cl:nil "# List of human tracking data~%HumanTrackingData[] humans~%~%================================================================================~%MSG: human_detection_pipeline/HumanTrackingData~%# Define a message for storing the position (x, y, z)~%geometry_msgs/Point position   # A standard ROS message type for a 3D point (x, y, z)~%~%# Tracking information for one human~%int32 tracking_id               # Unique identifier for the human~%float32 distance                # Distance to the human~%float32 confidence              # Confidence of the tracking data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumansList)))
  "Returns full string definition for message of type 'HumansList"
  (cl:format cl:nil "# List of human tracking data~%HumanTrackingData[] humans~%~%================================================================================~%MSG: human_detection_pipeline/HumanTrackingData~%# Define a message for storing the position (x, y, z)~%geometry_msgs/Point position   # A standard ROS message type for a 3D point (x, y, z)~%~%# Tracking information for one human~%int32 tracking_id               # Unique identifier for the human~%float32 distance                # Distance to the human~%float32 confidence              # Confidence of the tracking data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumansList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'humans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumansList>))
  "Converts a ROS message object to a list"
  (cl:list 'HumansList
    (cl:cons ':humans (humans msg))
))
