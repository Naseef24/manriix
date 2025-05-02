; Auto-generated. Do not edit!


(cl:in-package human_clustering_pipeline-msg)


;//! \htmlinclude OptimalPosition.msg.html

(cl:defclass <OptimalPosition> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (orientation
    :reader orientation
    :initarg :orientation
    :type cl:float
    :initform 0.0)
   (priority_score
    :reader priority_score
    :initarg :priority_score
    :type cl:float
    :initform 0.0))
)

(cl:defclass OptimalPosition (<OptimalPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OptimalPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OptimalPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name human_clustering_pipeline-msg:<OptimalPosition> is deprecated: use human_clustering_pipeline-msg:OptimalPosition instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <OptimalPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_clustering_pipeline-msg:x-val is deprecated.  Use human_clustering_pipeline-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <OptimalPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_clustering_pipeline-msg:y-val is deprecated.  Use human_clustering_pipeline-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <OptimalPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_clustering_pipeline-msg:orientation-val is deprecated.  Use human_clustering_pipeline-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'priority_score-val :lambda-list '(m))
(cl:defmethod priority_score-val ((m <OptimalPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_clustering_pipeline-msg:priority_score-val is deprecated.  Use human_clustering_pipeline-msg:priority_score instead.")
  (priority_score m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OptimalPosition>) ostream)
  "Serializes a message object of type '<OptimalPosition>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'orientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'priority_score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OptimalPosition>) istream)
  "Deserializes a message object of type '<OptimalPosition>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientation) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'priority_score) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OptimalPosition>)))
  "Returns string type for a message object of type '<OptimalPosition>"
  "human_clustering_pipeline/OptimalPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OptimalPosition)))
  "Returns string type for a message object of type 'OptimalPosition"
  "human_clustering_pipeline/OptimalPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OptimalPosition>)))
  "Returns md5sum for a message object of type '<OptimalPosition>"
  "8dfd8a1c68c8f6ee8f557969f62c7f98")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OptimalPosition)))
  "Returns md5sum for a message object of type 'OptimalPosition"
  "8dfd8a1c68c8f6ee8f557969f62c7f98")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OptimalPosition>)))
  "Returns full string definition for message of type '<OptimalPosition>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 orientation~%float64 priority_score~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OptimalPosition)))
  "Returns full string definition for message of type 'OptimalPosition"
  (cl:format cl:nil "float64 x~%float64 y~%float64 orientation~%float64 priority_score~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OptimalPosition>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OptimalPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'OptimalPosition
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':priority_score (priority_score msg))
))
