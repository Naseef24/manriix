; Auto-generated. Do not edit!


(cl:in-package human_detection_pipeline-msg)


;//! \htmlinclude HumanTrackingData.msg.html

(cl:defclass <HumanTrackingData> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (tracking_id
    :reader tracking_id
    :initarg :tracking_id
    :type cl:integer
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass HumanTrackingData (<HumanTrackingData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumanTrackingData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumanTrackingData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name human_detection_pipeline-msg:<HumanTrackingData> is deprecated: use human_detection_pipeline-msg:HumanTrackingData instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <HumanTrackingData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_detection_pipeline-msg:position-val is deprecated.  Use human_detection_pipeline-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'tracking_id-val :lambda-list '(m))
(cl:defmethod tracking_id-val ((m <HumanTrackingData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_detection_pipeline-msg:tracking_id-val is deprecated.  Use human_detection_pipeline-msg:tracking_id instead.")
  (tracking_id m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <HumanTrackingData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_detection_pipeline-msg:distance-val is deprecated.  Use human_detection_pipeline-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <HumanTrackingData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_detection_pipeline-msg:confidence-val is deprecated.  Use human_detection_pipeline-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumanTrackingData>) ostream)
  "Serializes a message object of type '<HumanTrackingData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let* ((signed (cl:slot-value msg 'tracking_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumanTrackingData>) istream)
  "Deserializes a message object of type '<HumanTrackingData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tracking_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumanTrackingData>)))
  "Returns string type for a message object of type '<HumanTrackingData>"
  "human_detection_pipeline/HumanTrackingData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumanTrackingData)))
  "Returns string type for a message object of type 'HumanTrackingData"
  "human_detection_pipeline/HumanTrackingData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumanTrackingData>)))
  "Returns md5sum for a message object of type '<HumanTrackingData>"
  "dc13d57981a0f76b7a28dbe20a53f4bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumanTrackingData)))
  "Returns md5sum for a message object of type 'HumanTrackingData"
  "dc13d57981a0f76b7a28dbe20a53f4bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumanTrackingData>)))
  "Returns full string definition for message of type '<HumanTrackingData>"
  (cl:format cl:nil "# Define a message for storing the position (x, y, z)~%geometry_msgs/Point position   # A standard ROS message type for a 3D point (x, y, z)~%~%# Tracking information for one human~%int32 tracking_id               # Unique identifier for the human~%float32 distance                # Distance to the human~%float32 confidence              # Confidence of the tracking data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumanTrackingData)))
  "Returns full string definition for message of type 'HumanTrackingData"
  (cl:format cl:nil "# Define a message for storing the position (x, y, z)~%geometry_msgs/Point position   # A standard ROS message type for a 3D point (x, y, z)~%~%# Tracking information for one human~%int32 tracking_id               # Unique identifier for the human~%float32 distance                # Distance to the human~%float32 confidence              # Confidence of the tracking data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumanTrackingData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumanTrackingData>))
  "Converts a ROS message object to a list"
  (cl:list 'HumanTrackingData
    (cl:cons ':position (position msg))
    (cl:cons ':tracking_id (tracking_id msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':confidence (confidence msg))
))
