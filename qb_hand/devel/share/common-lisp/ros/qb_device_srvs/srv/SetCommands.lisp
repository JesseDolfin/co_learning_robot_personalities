; Auto-generated. Do not edit!


(cl:in-package qb_device_srvs-srv)


;//! \htmlinclude SetCommands-request.msg.html

(cl:defclass <SetCommands-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (max_repeats
    :reader max_repeats
    :initarg :max_repeats
    :type cl:integer
    :initform 0)
   (set_commands
    :reader set_commands
    :initarg :set_commands
    :type cl:boolean
    :initform cl:nil)
   (set_commands_async
    :reader set_commands_async
    :initarg :set_commands_async
    :type cl:boolean
    :initform cl:nil)
   (commands
    :reader commands
    :initarg :commands
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass SetCommands-request (<SetCommands-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCommands-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCommands-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<SetCommands-request> is deprecated: use qb_device_srvs-srv:SetCommands-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SetCommands-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:id-val is deprecated.  Use qb_device_srvs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'max_repeats-val :lambda-list '(m))
(cl:defmethod max_repeats-val ((m <SetCommands-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:max_repeats-val is deprecated.  Use qb_device_srvs-srv:max_repeats instead.")
  (max_repeats m))

(cl:ensure-generic-function 'set_commands-val :lambda-list '(m))
(cl:defmethod set_commands-val ((m <SetCommands-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:set_commands-val is deprecated.  Use qb_device_srvs-srv:set_commands instead.")
  (set_commands m))

(cl:ensure-generic-function 'set_commands_async-val :lambda-list '(m))
(cl:defmethod set_commands_async-val ((m <SetCommands-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:set_commands_async-val is deprecated.  Use qb_device_srvs-srv:set_commands_async instead.")
  (set_commands_async m))

(cl:ensure-generic-function 'commands-val :lambda-list '(m))
(cl:defmethod commands-val ((m <SetCommands-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:commands-val is deprecated.  Use qb_device_srvs-srv:commands instead.")
  (commands m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCommands-request>) ostream)
  "Serializes a message object of type '<SetCommands-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'max_repeats)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_commands) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_commands_async) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'commands))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'commands))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCommands-request>) istream)
  "Deserializes a message object of type '<SetCommands-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_repeats) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'set_commands) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'set_commands_async) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'commands) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'commands)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCommands-request>)))
  "Returns string type for a service object of type '<SetCommands-request>"
  "qb_device_srvs/SetCommandsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommands-request)))
  "Returns string type for a service object of type 'SetCommands-request"
  "qb_device_srvs/SetCommandsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCommands-request>)))
  "Returns md5sum for a message object of type '<SetCommands-request>"
  "2a24c554c16e33a4da324c504c12f0f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCommands-request)))
  "Returns md5sum for a message object of type 'SetCommands-request"
  "2a24c554c16e33a4da324c504c12f0f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCommands-request>)))
  "Returns full string definition for message of type '<SetCommands-request>"
  (cl:format cl:nil "# request~%int32 id~%int32 max_repeats~%bool set_commands~%bool set_commands_async~%int16[] commands~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCommands-request)))
  "Returns full string definition for message of type 'SetCommands-request"
  (cl:format cl:nil "# request~%int32 id~%int32 max_repeats~%bool set_commands~%bool set_commands_async~%int16[] commands~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCommands-request>))
  (cl:+ 0
     4
     4
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'commands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCommands-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCommands-request
    (cl:cons ':id (id msg))
    (cl:cons ':max_repeats (max_repeats msg))
    (cl:cons ':set_commands (set_commands msg))
    (cl:cons ':set_commands_async (set_commands_async msg))
    (cl:cons ':commands (commands msg))
))
;//! \htmlinclude SetCommands-response.msg.html

(cl:defclass <SetCommands-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (failures
    :reader failures
    :initarg :failures
    :type cl:integer
    :initform 0))
)

(cl:defclass SetCommands-response (<SetCommands-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCommands-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCommands-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<SetCommands-response> is deprecated: use qb_device_srvs-srv:SetCommands-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetCommands-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:success-val is deprecated.  Use qb_device_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'failures-val :lambda-list '(m))
(cl:defmethod failures-val ((m <SetCommands-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:failures-val is deprecated.  Use qb_device_srvs-srv:failures instead.")
  (failures m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCommands-response>) ostream)
  "Serializes a message object of type '<SetCommands-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'failures)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCommands-response>) istream)
  "Deserializes a message object of type '<SetCommands-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'failures) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCommands-response>)))
  "Returns string type for a service object of type '<SetCommands-response>"
  "qb_device_srvs/SetCommandsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommands-response)))
  "Returns string type for a service object of type 'SetCommands-response"
  "qb_device_srvs/SetCommandsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCommands-response>)))
  "Returns md5sum for a message object of type '<SetCommands-response>"
  "2a24c554c16e33a4da324c504c12f0f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCommands-response)))
  "Returns md5sum for a message object of type 'SetCommands-response"
  "2a24c554c16e33a4da324c504c12f0f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCommands-response>)))
  "Returns full string definition for message of type '<SetCommands-response>"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCommands-response)))
  "Returns full string definition for message of type 'SetCommands-response"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCommands-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCommands-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCommands-response
    (cl:cons ':success (success msg))
    (cl:cons ':failures (failures msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCommands)))
  'SetCommands-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCommands)))
  'SetCommands-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommands)))
  "Returns string type for a service object of type '<SetCommands>"
  "qb_device_srvs/SetCommands")