; Auto-generated. Do not edit!


(cl:in-package qb_device_msgs-msg)


;//! \htmlinclude ConnectionState.msg.html

(cl:defclass <ConnectionState> (roslisp-msg-protocol:ros-message)
  ((devices
    :reader devices
    :initarg :devices
    :type (cl:vector qb_device_msgs-msg:DeviceConnectionInfo)
   :initform (cl:make-array 0 :element-type 'qb_device_msgs-msg:DeviceConnectionInfo :initial-element (cl:make-instance 'qb_device_msgs-msg:DeviceConnectionInfo))))
)

(cl:defclass ConnectionState (<ConnectionState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConnectionState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConnectionState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_msgs-msg:<ConnectionState> is deprecated: use qb_device_msgs-msg:ConnectionState instead.")))

(cl:ensure-generic-function 'devices-val :lambda-list '(m))
(cl:defmethod devices-val ((m <ConnectionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:devices-val is deprecated.  Use qb_device_msgs-msg:devices instead.")
  (devices m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConnectionState>) ostream)
  "Serializes a message object of type '<ConnectionState>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'devices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'devices))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConnectionState>) istream)
  "Deserializes a message object of type '<ConnectionState>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'devices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'devices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qb_device_msgs-msg:DeviceConnectionInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConnectionState>)))
  "Returns string type for a message object of type '<ConnectionState>"
  "qb_device_msgs/ConnectionState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectionState)))
  "Returns string type for a message object of type 'ConnectionState"
  "qb_device_msgs/ConnectionState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConnectionState>)))
  "Returns md5sum for a message object of type '<ConnectionState>"
  "3fb8aec8a16b727f8528545cb7e04895")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConnectionState)))
  "Returns md5sum for a message object of type 'ConnectionState"
  "3fb8aec8a16b727f8528545cb7e04895")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConnectionState>)))
  "Returns full string definition for message of type '<ConnectionState>"
  (cl:format cl:nil "# Device-independent connection state message~%~%qb_device_msgs/DeviceConnectionInfo[] devices~%================================================================================~%MSG: qb_device_msgs/DeviceConnectionInfo~%# Device-independent message that constains: ~%~%int32 id           # device id;~%bool is_active     # motor activation status;~%string port        # serial port~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConnectionState)))
  "Returns full string definition for message of type 'ConnectionState"
  (cl:format cl:nil "# Device-independent connection state message~%~%qb_device_msgs/DeviceConnectionInfo[] devices~%================================================================================~%MSG: qb_device_msgs/DeviceConnectionInfo~%# Device-independent message that constains: ~%~%int32 id           # device id;~%bool is_active     # motor activation status;~%string port        # serial port~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConnectionState>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'devices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConnectionState>))
  "Converts a ROS message object to a list"
  (cl:list 'ConnectionState
    (cl:cons ':devices (devices msg))
))
