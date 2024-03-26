; Auto-generated. Do not edit!


(cl:in-package qb_device_msgs-msg)


;//! \htmlinclude State.msg.html

(cl:defclass <State> (roslisp-msg-protocol:ros-message)
  ((actuators
    :reader actuators
    :initarg :actuators
    :type (cl:vector qb_device_msgs-msg:ResourceData)
   :initform (cl:make-array 0 :element-type 'qb_device_msgs-msg:ResourceData :initial-element (cl:make-instance 'qb_device_msgs-msg:ResourceData)))
   (joints
    :reader joints
    :initarg :joints
    :type (cl:vector qb_device_msgs-msg:ResourceData)
   :initform (cl:make-array 0 :element-type 'qb_device_msgs-msg:ResourceData :initial-element (cl:make-instance 'qb_device_msgs-msg:ResourceData)))
   (is_reliable
    :reader is_reliable
    :initarg :is_reliable
    :type cl:boolean
    :initform cl:nil)
   (consecutive_failures
    :reader consecutive_failures
    :initarg :consecutive_failures
    :type cl:integer
    :initform 0))
)

(cl:defclass State (<State>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <State>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'State)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_msgs-msg:<State> is deprecated: use qb_device_msgs-msg:State instead.")))

(cl:ensure-generic-function 'actuators-val :lambda-list '(m))
(cl:defmethod actuators-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:actuators-val is deprecated.  Use qb_device_msgs-msg:actuators instead.")
  (actuators m))

(cl:ensure-generic-function 'joints-val :lambda-list '(m))
(cl:defmethod joints-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:joints-val is deprecated.  Use qb_device_msgs-msg:joints instead.")
  (joints m))

(cl:ensure-generic-function 'is_reliable-val :lambda-list '(m))
(cl:defmethod is_reliable-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:is_reliable-val is deprecated.  Use qb_device_msgs-msg:is_reliable instead.")
  (is_reliable m))

(cl:ensure-generic-function 'consecutive_failures-val :lambda-list '(m))
(cl:defmethod consecutive_failures-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:consecutive_failures-val is deprecated.  Use qb_device_msgs-msg:consecutive_failures instead.")
  (consecutive_failures m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <State>) ostream)
  "Serializes a message object of type '<State>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'actuators))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'actuators))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'joints))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_reliable) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'consecutive_failures)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <State>) istream)
  "Deserializes a message object of type '<State>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'actuators) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'actuators)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qb_device_msgs-msg:ResourceData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qb_device_msgs-msg:ResourceData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'is_reliable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'consecutive_failures) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<State>)))
  "Returns string type for a message object of type '<State>"
  "qb_device_msgs/State")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'State)))
  "Returns string type for a message object of type 'State"
  "qb_device_msgs/State")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<State>)))
  "Returns md5sum for a message object of type '<State>"
  "035992012f0af1c782c17a0f8f6e544c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'State)))
  "Returns md5sum for a message object of type 'State"
  "035992012f0af1c782c17a0f8f6e544c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<State>)))
  "Returns full string definition for message of type '<State>"
  (cl:format cl:nil "# State message valid for either qbhand or qbmove~%~%# either qbhand or qbmove:~%#  - motors: position, command in [ticks], velocity in [ticks/s], effort in [mA]~%qb_device_msgs/ResourceData[] actuators~%~%# qbhand:~%#  - closure: position, command in [0,1], velocity in [percent/s],  effort in [A].~%# qbmove:~%#  - shaft: position, command in [radians], velocity in [radians/s], effort in [A];~%#  - preset: position, command in [0,1], velocity in [percent/s], effort is not used.~%qb_device_msgs/ResourceData[] joints~%~%# Reliability of the retrieved measurements~%bool is_reliable~%int32 consecutive_failures~%================================================================================~%MSG: qb_device_msgs/ResourceData~%# Device-independent resource data message~%~%string name~%float64 position~%float64 velocity~%float64 effort~%float64 command~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'State)))
  "Returns full string definition for message of type 'State"
  (cl:format cl:nil "# State message valid for either qbhand or qbmove~%~%# either qbhand or qbmove:~%#  - motors: position, command in [ticks], velocity in [ticks/s], effort in [mA]~%qb_device_msgs/ResourceData[] actuators~%~%# qbhand:~%#  - closure: position, command in [0,1], velocity in [percent/s],  effort in [A].~%# qbmove:~%#  - shaft: position, command in [radians], velocity in [radians/s], effort in [A];~%#  - preset: position, command in [0,1], velocity in [percent/s], effort is not used.~%qb_device_msgs/ResourceData[] joints~%~%# Reliability of the retrieved measurements~%bool is_reliable~%int32 consecutive_failures~%================================================================================~%MSG: qb_device_msgs/ResourceData~%# Device-independent resource data message~%~%string name~%float64 position~%float64 velocity~%float64 effort~%float64 command~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <State>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'actuators) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <State>))
  "Converts a ROS message object to a list"
  (cl:list 'State
    (cl:cons ':actuators (actuators msg))
    (cl:cons ':joints (joints msg))
    (cl:cons ':is_reliable (is_reliable msg))
    (cl:cons ':consecutive_failures (consecutive_failures msg))
))
