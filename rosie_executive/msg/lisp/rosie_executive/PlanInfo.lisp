; Auto-generated. Do not edit!


(in-package rosie_executive-msg)


;//! \htmlinclude PlanInfo.msg.html

(defclass <PlanInfo> (ros-message)
  ((obj_type
    :reader obj_type-val
    :initarg :obj_type
    :type string
    :initform "")
   (obj_location
    :reader obj_location-val
    :initarg :obj_location
    :type geometry_msgs-msg:<PoseStamped>
    :initform (make-instance 'geometry_msgs-msg:<PoseStamped>))
   (robot_location
    :reader robot_location-val
    :initarg :robot_location
    :type geometry_msgs-msg:<PoseStamped>
    :initform (make-instance 'geometry_msgs-msg:<PoseStamped>))
   (status
    :reader status-val
    :initarg :status
    :type string
    :initform ""))
)
(defmethod serialize ((msg <PlanInfo>) ostream)
  "Serializes a message object of type '<PlanInfo>"
  (let ((__ros_str_len (length (slot-value msg 'obj_type))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'obj_type))
  (serialize (slot-value msg 'obj_location) ostream)
  (serialize (slot-value msg 'robot_location) ostream)
  (let ((__ros_str_len (length (slot-value msg 'status))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'status))
)
(defmethod deserialize ((msg <PlanInfo>) istream)
  "Deserializes a message object of type '<PlanInfo>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'obj_type) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'obj_type) __ros_str_idx) (code-char (read-byte istream)))))
  (deserialize (slot-value msg 'obj_location) istream)
  (deserialize (slot-value msg 'robot_location) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'status) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'status) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<PlanInfo>)))
  "Returns string type for a message object of type '<PlanInfo>"
  "rosie_executive/PlanInfo")
(defmethod md5sum ((type (eql '<PlanInfo>)))
  "Returns md5sum for a message object of type '<PlanInfo>"
  "4345b171f98bf50efb9e3ba70df824e6")
(defmethod message-definition ((type (eql '<PlanInfo>)))
  "Returns full string definition for message of type '<PlanInfo>"
  (format nil "string obj_type~%geometry_msgs/PoseStamped obj_location          # The pose of the object being manipulated~%geometry_msgs/PoseStamped robot_location        # The pose of the robot during the action~%string status                                   # Status of the plan (SUCCEEDED, FAILED, EVABORATED)~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(defmethod serialization-length ((msg <PlanInfo>))
  (+ 0
     4 (length (slot-value msg 'obj_type))
     (serialization-length (slot-value msg 'obj_location))
     (serialization-length (slot-value msg 'robot_location))
     4 (length (slot-value msg 'status))
))
(defmethod ros-message-to-list ((msg <PlanInfo>))
  "Converts a ROS message object to a list"
  (list '<PlanInfo>
    (cons ':obj_type (obj_type-val msg))
    (cons ':obj_location (obj_location-val msg))
    (cons ':robot_location (robot_location-val msg))
    (cons ':status (status-val msg))
))
