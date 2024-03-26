; Auto-generated. Do not edit!


(cl:in-package qb_device_msgs-msg)


;//! \htmlinclude DeviceConnectionInfo.msg.html

(cl:defclass <DeviceConnectionInfo> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (is_active
    :reader is_active
    :initarg :is_active
    :type cl:boolean
    :initform cl:nil)
   (port
    :reader port
    :initarg :port
    :type cl:string
    :initform ""))
)

(cl:defclass DeviceConnectionInfo (<DeviceConnectionInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceConnectionInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceConnectionInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_msgs-msg:<DeviceConnectionInfo> is deprecated: use qb_device_msgs-msg:DeviceConnectionInfo instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <DeviceConnectionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:id-val is deprecated.  Use qb_device_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'is_active-val :lambda-list '(m))
(cl:defmethod is_active-val ((m <DeviceConnectionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:is_active-val is deprecated.  Use qb_device_msgs-msg:is_active instead.")
  (is_active m))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <DeviceConnectionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_msgs-msg:port-val is deprecated.  Use qb_device_msgs-msg:port instead.")
  (port m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceConnectionInfo>) ostream)
  "Serializes a message object of type '<DeviceConnectionInfo>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_active) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'port))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'port))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceConnectionInfo>) istream)
  "Deserializes a message object of type '<DeviceConnectionInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'is_active) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'port) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceConnectionInfo>)))
  "Returns string type for a message object of type '<DeviceConnectionInfo>"
  "qb_device_msgs/DeviceConnectionInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceConnectionInfo)))
  "Returns string type for a message object of type 'DeviceConnectionInfo"
  "qb_device_msgs/DeviceConnectionInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceConnectionInfo>)))
  "Returns md5sum for a message object of type '<DeviceConnectionInfo>"
  "1a1c593244281b064cf77ab64e673fe3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceConnectionInfo)))
  "Returns md5sum for a message object of type 'DeviceConnectionInfo"
  "1a1c593244281b064cf77ab64e673fe3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceConnectionInfo>)))
  "Returns full string definition for message of type '<DeviceConnectionInfo>"
  (cl:format cl:nil "# Device-independent message that constains: ~%~%int32 id           # device id;~%bool is_active     # motor activation status;~%string port        # serial port~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceConnectionInfo)))
  "Returns full string definition for message of type 'DeviceConnectionInfo"
  (cl:format cl:nil "# Device-independent message that constains: ~%~%int32 id           # device id;~%bool is_active     # motor activation status;~%string port        # serial port~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceConnectionInfo>))
  (cl:+ 0
     4
     1
     4 (cl:length (cl:slot-value msg 'port))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceConnectionInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceConnectionInfo
    (cl:cons ':id (id msg))
    (cl:cons ':is_active (is_active msg))
    (cl:cons ':port (port msg))
))
