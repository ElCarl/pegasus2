; Auto-generated. Do not edit!


(cl:in-package arduino_communicator-msg)


;//! \htmlinclude EncoderCounts.msg.html

(cl:defclass <EncoderCounts> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_wheel_counts
    :reader left_wheel_counts
    :initarg :left_wheel_counts
    :type (cl:vector cl:integer)
   :initform (cl:make-array 3 :element-type 'cl:integer :initial-element 0))
   (right_wheel_counts
    :reader right_wheel_counts
    :initarg :right_wheel_counts
    :type (cl:vector cl:integer)
   :initform (cl:make-array 3 :element-type 'cl:integer :initial-element 0))
   (base_rotation_counts
    :reader base_rotation_counts
    :initarg :base_rotation_counts
    :type cl:integer
    :initform 0)
   (wrist_rotation_counts
    :reader wrist_rotation_counts
    :initarg :wrist_rotation_counts
    :type cl:integer
    :initform 0)
   (gripper_counts
    :reader gripper_counts
    :initarg :gripper_counts
    :type cl:integer
    :initform 0))
)

(cl:defclass EncoderCounts (<EncoderCounts>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EncoderCounts>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EncoderCounts)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arduino_communicator-msg:<EncoderCounts> is deprecated: use arduino_communicator-msg:EncoderCounts instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EncoderCounts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arduino_communicator-msg:header-val is deprecated.  Use arduino_communicator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_wheel_counts-val :lambda-list '(m))
(cl:defmethod left_wheel_counts-val ((m <EncoderCounts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arduino_communicator-msg:left_wheel_counts-val is deprecated.  Use arduino_communicator-msg:left_wheel_counts instead.")
  (left_wheel_counts m))

(cl:ensure-generic-function 'right_wheel_counts-val :lambda-list '(m))
(cl:defmethod right_wheel_counts-val ((m <EncoderCounts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arduino_communicator-msg:right_wheel_counts-val is deprecated.  Use arduino_communicator-msg:right_wheel_counts instead.")
  (right_wheel_counts m))

(cl:ensure-generic-function 'base_rotation_counts-val :lambda-list '(m))
(cl:defmethod base_rotation_counts-val ((m <EncoderCounts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arduino_communicator-msg:base_rotation_counts-val is deprecated.  Use arduino_communicator-msg:base_rotation_counts instead.")
  (base_rotation_counts m))

(cl:ensure-generic-function 'wrist_rotation_counts-val :lambda-list '(m))
(cl:defmethod wrist_rotation_counts-val ((m <EncoderCounts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arduino_communicator-msg:wrist_rotation_counts-val is deprecated.  Use arduino_communicator-msg:wrist_rotation_counts instead.")
  (wrist_rotation_counts m))

(cl:ensure-generic-function 'gripper_counts-val :lambda-list '(m))
(cl:defmethod gripper_counts-val ((m <EncoderCounts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arduino_communicator-msg:gripper_counts-val is deprecated.  Use arduino_communicator-msg:gripper_counts instead.")
  (gripper_counts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EncoderCounts>) ostream)
  "Serializes a message object of type '<EncoderCounts>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'left_wheel_counts))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'right_wheel_counts))
  (cl:let* ((signed (cl:slot-value msg 'base_rotation_counts)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'wrist_rotation_counts)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gripper_counts)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EncoderCounts>) istream)
  "Deserializes a message object of type '<EncoderCounts>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'left_wheel_counts) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'left_wheel_counts)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'right_wheel_counts) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'right_wheel_counts)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'base_rotation_counts) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'wrist_rotation_counts) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gripper_counts) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EncoderCounts>)))
  "Returns string type for a message object of type '<EncoderCounts>"
  "arduino_communicator/EncoderCounts")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EncoderCounts)))
  "Returns string type for a message object of type 'EncoderCounts"
  "arduino_communicator/EncoderCounts")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EncoderCounts>)))
  "Returns md5sum for a message object of type '<EncoderCounts>"
  "890e34750b24fc194afb98e978102a4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EncoderCounts)))
  "Returns md5sum for a message object of type 'EncoderCounts"
  "890e34750b24fc194afb98e978102a4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EncoderCounts>)))
  "Returns full string definition for message of type '<EncoderCounts>"
  (cl:format cl:nil "Header header~%int32[3] left_wheel_counts   # Front to back~%int32[3] right_wheel_counts  # Front to back~%int32 base_rotation_counts~%int32 wrist_rotation_counts~%int32 gripper_counts~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EncoderCounts)))
  "Returns full string definition for message of type 'EncoderCounts"
  (cl:format cl:nil "Header header~%int32[3] left_wheel_counts   # Front to back~%int32[3] right_wheel_counts  # Front to back~%int32 base_rotation_counts~%int32 wrist_rotation_counts~%int32 gripper_counts~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EncoderCounts>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'left_wheel_counts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'right_wheel_counts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EncoderCounts>))
  "Converts a ROS message object to a list"
  (cl:list 'EncoderCounts
    (cl:cons ':header (header msg))
    (cl:cons ':left_wheel_counts (left_wheel_counts msg))
    (cl:cons ':right_wheel_counts (right_wheel_counts msg))
    (cl:cons ':base_rotation_counts (base_rotation_counts msg))
    (cl:cons ':wrist_rotation_counts (wrist_rotation_counts msg))
    (cl:cons ':gripper_counts (gripper_counts msg))
))
