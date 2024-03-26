; Auto-generated. Do not edit!


(cl:in-package qb_device_srvs-srv)


;//! \htmlinclude InitializeDevice-request.msg.html

(cl:defclass <InitializeDevice-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (activate
    :reader activate
    :initarg :activate
    :type cl:boolean
    :initform cl:nil)
   (rescan
    :reader rescan
    :initarg :rescan
    :type cl:boolean
    :initform cl:nil)
   (max_repeats
    :reader max_repeats
    :initarg :max_repeats
    :type cl:integer
    :initform 0))
)

(cl:defclass InitializeDevice-request (<InitializeDevice-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitializeDevice-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitializeDevice-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<InitializeDevice-request> is deprecated: use qb_device_srvs-srv:InitializeDevice-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <InitializeDevice-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:id-val is deprecated.  Use qb_device_srvs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'activate-val :lambda-list '(m))
(cl:defmethod activate-val ((m <InitializeDevice-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:activate-val is deprecated.  Use qb_device_srvs-srv:activate instead.")
  (activate m))

(cl:ensure-generic-function 'rescan-val :lambda-list '(m))
(cl:defmethod rescan-val ((m <InitializeDevice-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:rescan-val is deprecated.  Use qb_device_srvs-srv:rescan instead.")
  (rescan m))

(cl:ensure-generic-function 'max_repeats-val :lambda-list '(m))
(cl:defmethod max_repeats-val ((m <InitializeDevice-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:max_repeats-val is deprecated.  Use qb_device_srvs-srv:max_repeats instead.")
  (max_repeats m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitializeDevice-request>) ostream)
  "Serializes a message object of type '<InitializeDevice-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'activate) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rescan) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'max_repeats)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitializeDevice-request>) istream)
  "Deserializes a message object of type '<InitializeDevice-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'activate) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'rescan) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_repeats) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitializeDevice-request>)))
  "Returns string type for a service object of type '<InitializeDevice-request>"
  "qb_device_srvs/InitializeDeviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitializeDevice-request)))
  "Returns string type for a service object of type 'InitializeDevice-request"
  "qb_device_srvs/InitializeDeviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitializeDevice-request>)))
  "Returns md5sum for a message object of type '<InitializeDevice-request>"
  "c23409117bb3e9fa24111fdf884ca1f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitializeDevice-request)))
  "Returns md5sum for a message object of type 'InitializeDevice-request"
  "c23409117bb3e9fa24111fdf884ca1f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitializeDevice-request>)))
  "Returns full string definition for message of type '<InitializeDevice-request>"
  (cl:format cl:nil "# request~%int32 id~%bool activate~%bool rescan~%int32 max_repeats~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitializeDevice-request)))
  "Returns full string definition for message of type 'InitializeDevice-request"
  (cl:format cl:nil "# request~%int32 id~%bool activate~%bool rescan~%int32 max_repeats~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitializeDevice-request>))
  (cl:+ 0
     4
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitializeDevice-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InitializeDevice-request
    (cl:cons ':id (id msg))
    (cl:cons ':activate (activate msg))
    (cl:cons ':rescan (rescan msg))
    (cl:cons ':max_repeats (max_repeats msg))
))
;//! \htmlinclude InitializeDevice-response.msg.html

(cl:defclass <InitializeDevice-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (failures
    :reader failures
    :initarg :failures
    :type cl:integer
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (info
    :reader info
    :initarg :info
    :type qb_device_msgs-msg:Info
    :initform (cl:make-instance 'qb_device_msgs-msg:Info)))
)

(cl:defclass InitializeDevice-response (<InitializeDevice-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitializeDevice-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitializeDevice-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<InitializeDevice-response> is deprecated: use qb_device_srvs-srv:InitializeDevice-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <InitializeDevice-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:success-val is deprecated.  Use qb_device_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'failures-val :lambda-list '(m))
(cl:defmethod failures-val ((m <InitializeDevice-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:failures-val is deprecated.  Use qb_device_srvs-srv:failures instead.")
  (failures m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <InitializeDevice-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:message-val is deprecated.  Use qb_device_srvs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <InitializeDevice-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:info-val is deprecated.  Use qb_device_srvs-srv:info instead.")
  (info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitializeDevice-response>) ostream)
  "Serializes a message object of type '<InitializeDevice-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'failures)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'info) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitializeDevice-response>) istream)
  "Deserializes a message object of type '<InitializeDevice-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'failures) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'info) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitializeDevice-response>)))
  "Returns string type for a service object of type '<InitializeDevice-response>"
  "qb_device_srvs/InitializeDeviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitializeDevice-response)))
  "Returns string type for a service object of type 'InitializeDevice-response"
  "qb_device_srvs/InitializeDeviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitializeDevice-response>)))
  "Returns md5sum for a message object of type '<InitializeDevice-response>"
  "c23409117bb3e9fa24111fdf884ca1f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitializeDevice-response)))
  "Returns md5sum for a message object of type 'InitializeDevice-response"
  "c23409117bb3e9fa24111fdf884ca1f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitializeDevice-response>)))
  "Returns full string definition for message of type '<InitializeDevice-response>"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%string message~%qb_device_msgs/Info info~%~%================================================================================~%MSG: qb_device_msgs/Info~%# Standard device-independent info message~%~%int32 id~%string serial_port~%int32 max_repeats~%bool get_positions~%bool get_currents~%bool get_distinct_packages~%bool set_commands~%bool set_commands_async~%int32[] position_limits~%uint8[] encoder_resolutions~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitializeDevice-response)))
  "Returns full string definition for message of type 'InitializeDevice-response"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%string message~%qb_device_msgs/Info info~%~%================================================================================~%MSG: qb_device_msgs/Info~%# Standard device-independent info message~%~%int32 id~%string serial_port~%int32 max_repeats~%bool get_positions~%bool get_currents~%bool get_distinct_packages~%bool set_commands~%bool set_commands_async~%int32[] position_limits~%uint8[] encoder_resolutions~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitializeDevice-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'message))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitializeDevice-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InitializeDevice-response
    (cl:cons ':success (success msg))
    (cl:cons ':failures (failures msg))
    (cl:cons ':message (message msg))
    (cl:cons ':info (info msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InitializeDevice)))
  'InitializeDevice-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InitializeDevice)))
  'InitializeDevice-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitializeDevice)))
  "Returns string type for a service object of type '<InitializeDevice>"
  "qb_device_srvs/InitializeDevice")