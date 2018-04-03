;; Auto-generated. Do not edit!


(when (boundp 'arduino_communicator::EncoderCounts)
  (if (not (find-package "ARDUINO_COMMUNICATOR"))
    (make-package "ARDUINO_COMMUNICATOR"))
  (shadow 'EncoderCounts (find-package "ARDUINO_COMMUNICATOR")))
(unless (find-package "ARDUINO_COMMUNICATOR::ENCODERCOUNTS")
  (make-package "ARDUINO_COMMUNICATOR::ENCODERCOUNTS"))

(in-package "ROS")
;;//! \htmlinclude EncoderCounts.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass arduino_communicator::EncoderCounts
  :super ros::object
  :slots (_header _left_wheel_counts _right_wheel_counts _base_rotation_counts _wrist_rotation_counts _gripper_counts ))

(defmethod arduino_communicator::EncoderCounts
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_wheel_counts __left_wheel_counts) (make-array 3 :initial-element 0 :element-type :integer))
    ((:right_wheel_counts __right_wheel_counts) (make-array 3 :initial-element 0 :element-type :integer))
    ((:base_rotation_counts __base_rotation_counts) 0)
    ((:wrist_rotation_counts __wrist_rotation_counts) 0)
    ((:gripper_counts __gripper_counts) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_wheel_counts __left_wheel_counts)
   (setq _right_wheel_counts __right_wheel_counts)
   (setq _base_rotation_counts (round __base_rotation_counts))
   (setq _wrist_rotation_counts (round __wrist_rotation_counts))
   (setq _gripper_counts (round __gripper_counts))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_wheel_counts
   (&optional __left_wheel_counts)
   (if __left_wheel_counts (setq _left_wheel_counts __left_wheel_counts)) _left_wheel_counts)
  (:right_wheel_counts
   (&optional __right_wheel_counts)
   (if __right_wheel_counts (setq _right_wheel_counts __right_wheel_counts)) _right_wheel_counts)
  (:base_rotation_counts
   (&optional __base_rotation_counts)
   (if __base_rotation_counts (setq _base_rotation_counts __base_rotation_counts)) _base_rotation_counts)
  (:wrist_rotation_counts
   (&optional __wrist_rotation_counts)
   (if __wrist_rotation_counts (setq _wrist_rotation_counts __wrist_rotation_counts)) _wrist_rotation_counts)
  (:gripper_counts
   (&optional __gripper_counts)
   (if __gripper_counts (setq _gripper_counts __gripper_counts)) _gripper_counts)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32[3] _left_wheel_counts
    (* 4    3)
    ;; int32[3] _right_wheel_counts
    (* 4    3)
    ;; int32 _base_rotation_counts
    4
    ;; int32 _wrist_rotation_counts
    4
    ;; int32 _gripper_counts
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32[3] _left_wheel_counts
     (dotimes (i 3)
       (write-long (elt _left_wheel_counts i) s)
       )
     ;; int32[3] _right_wheel_counts
     (dotimes (i 3)
       (write-long (elt _right_wheel_counts i) s)
       )
     ;; int32 _base_rotation_counts
       (write-long _base_rotation_counts s)
     ;; int32 _wrist_rotation_counts
       (write-long _wrist_rotation_counts s)
     ;; int32 _gripper_counts
       (write-long _gripper_counts s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32[3] _left_wheel_counts
   (dotimes (i (length _left_wheel_counts))
     (setf (elt _left_wheel_counts i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32[3] _right_wheel_counts
   (dotimes (i (length _right_wheel_counts))
     (setf (elt _right_wheel_counts i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32 _base_rotation_counts
     (setq _base_rotation_counts (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _wrist_rotation_counts
     (setq _wrist_rotation_counts (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _gripper_counts
     (setq _gripper_counts (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get arduino_communicator::EncoderCounts :md5sum-) "890e34750b24fc194afb98e978102a4f")
(setf (get arduino_communicator::EncoderCounts :datatype-) "arduino_communicator/EncoderCounts")
(setf (get arduino_communicator::EncoderCounts :definition-)
      "Header header
int32[3] left_wheel_counts   # Front to back
int32[3] right_wheel_counts  # Front to back
int32 base_rotation_counts
int32 wrist_rotation_counts
int32 gripper_counts
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :arduino_communicator/EncoderCounts "890e34750b24fc194afb98e978102a4f")


