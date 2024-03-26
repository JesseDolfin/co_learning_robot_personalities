; Auto-generated. Do not edit!


(cl:in-package qb_device_msgs-msg)


;//! \htmlinclude StateStamped.msg.html

(cl:defclass <StateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (device_info
    :reader device_info
    :initarg :device_info
    :type qb_device_msgs-msg:Info
    :initform (cl:make-instance 'qb_device_msgs-msg:Info))
   (device_data
    :reader device_data
    :initarg :device_data
    :type qb_device_msgs-msg:State
    :initform (cl:make-instance 'qb_device_msgs-msg:State)))
)

(cl:defclass StateStamped (<StateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_msgs-msg:<StateStamped> is deprecated: use qb_device_msgs-msg:StateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:header-val is deprecated.  Use qb_device_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'device_info-val :lambda-list '(m))
(cl:defmethod device_info-val ((m <StateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:device_info-val is deprecated.  Use qb_device_msgs-msg:device_info instead.")
  (device_info m))

(cl:ensure-generic-function 'device_data-val :lambda-list '(m))
(cl:defmethod device_data-val ((m <StateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:device_data-val is deprecated.  Use qb_device_msgs-msg:device_data instead.")
  (device_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateStamped>) ostream)
  "Serializes a message object of type '<StateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'device_info) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'device_data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateStamped>) istream)
  "Deserializes a message object of type '<StateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'device_info) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'device_data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateStamped>)))
  "Returns string type for a message object of type '<StateStamped>"
  "qb_device_msgs/StateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateStamped)))
  "Returns string type for a message object of type 'StateStamped"
  "qb_device_msgs/StateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateStamped>)))
  "Returns md5sum for a message object of type '<StateStamped>"
  "4ad56bd88424f6cfda763bbf1b38cce8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateStamped)))
  "Returns md5sum for a message object of type 'StateStamped"
  "4ad56bd88424f6cfda763bbf1b38cce8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateStamped>)))
  "Returns full string definition for message of type '<StateStamped>"
  (cl:format cl:nil "# State message with stamped info valid for either qbhand or qbmove~%~%std_msgs/Header header~%~%qb_device_msgs/Info device_info~%~%qb_device_msgs/State device_data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: qb_device_msgs/Info~%# Standard device-independent info message~%~%int32 id~%string serial_port~%int32 max_repeats~%bool get_positions~%bool get_currents~%bool get_distinct_packages~%bool set_commands~%bool set_commands_async~%int32[] position_limits~%uint8[] encoder_resolutions~%================================================================================~%MSG: qb_device_msgs/State~%# State message valid for either qbhand or qbmove~%~%# either qbhand or qbmove:~%#  - motors: position, command in [ticks], velocity in [ticks/s], effort in [mA]~%qb_device_msgs/ResourceData[] actuators~%~%# qbhand:~%#  - closure: position, command in [0,1], velocity in [percent/s],  effort in [A].~%# qbmove:~%#  - shaft: position, command in [radians], velocity in [radians/s], effort in [A];~%#  - preset: position, command in [0,1], velocity in [percent/s], effort is not used.~%qb_device_msgs/ResourceData[] joints~%~%# Reliability of the retrieved measurements~%bool is_reliable~%int32 consecutive_failures~%================================================================================~%MSG: qb_device_msgs/ResourceData~%# Device-independent resource data message~%~%string name~%float64 position~%float64 velocity~%float64 effort~%float64 command~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateStamped)))
  "Returns full string definition for message of type 'StateStamped"
  (cl:format cl:nil "# State message with stamped info valid for either qbhand or qbmove~%~%std_msgs/Header header~%~%qb_device_msgs/Info device_info~%~%qb_device_msgs/State device_data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: qb_device_msgs/Info~%# Standard device-independent info message~%~%int32 id~%string serial_port~%int32 max_repeats~%bool get_positions~%bool get_currents~%bool get_distinct_packages~%bool set_commands~%bool set_commands_async~%int32[] position_limits~%uint8[] encoder_resolutions~%================================================================================~%MSG: qb_device_msgs/State~%# State message valid for either qbhand or qbmove~%~%# either qbhand or qbmove:~%#  - motors: position, command in [ticks], velocity in [ticks/s], effort in [mA]~%qb_device_msgs/ResourceData[] actuators~%~%# qbhand:~%#  - closure: position, command in [0,1], velocity in [percent/s],  effort in [A].~%# qbmove:~%#  - shaft: position, command in [radians], velocity in [radians/s], effort in [A];~%#  - preset: position, command in [0,1], velocity in [percent/s], effort is not used.~%qb_device_msgs/ResourceData[] joints~%~%# Reliability of the retrieved measurements~%bool is_reliable~%int32 consecutive_failures~%================================================================================~%MSG: qb_device_msgs/ResourceData~%# Device-independent resource data message~%~%string name~%float64 position~%float64 velocity~%float64 effort~%float64 command~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'device_info))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'device_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'StateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':device_info (device_info msg))
    (cl:cons ':device_data (device_data msg))
))
