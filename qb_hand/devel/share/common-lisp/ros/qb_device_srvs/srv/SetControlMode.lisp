; Auto-generated. Do not edit!


(cl:in-package qb_device_srvs-srv)


;//! \htmlinclude SetControlMode-request.msg.html

(cl:defclass <SetControlMode-request> (roslisp-msg-protocol:ros-message)
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
   (control
    :reader control
    :initarg :control
    :type cl:string
    :initform ""))
)

(cl:defclass SetControlMode-request (<SetControlMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<SetControlMode-request> is deprecated: use qb_device_srvs-srv:SetControlMode-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SetControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:id-val is deprecated.  Use qb_device_srvs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'max_repeats-val :lambda-list '(m))
(cl:defmethod max_repeats-val ((m <SetControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:max_repeats-val is deprecated.  Use qb_device_srvs-srv:max_repeats instead.")
  (max_repeats m))

(cl:ensure-generic-function 'control-val :lambda-list '(m))
(cl:defmethod control-val ((m <SetControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:control-val is deprecated.  Use qb_device_srvs-srv:control instead.")
  (control m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlMode-request>) ostream)
  "Serializes a message object of type '<SetControlMode-request>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'control))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'control))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlMode-request>) istream)
  "Deserializes a message object of type '<SetControlMode-request>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'control) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlMode-request>)))
  "Returns string type for a service object of type '<SetControlMode-request>"
  "qb_device_srvs/SetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode-request)))
  "Returns string type for a service object of type 'SetControlMode-request"
  "qb_device_srvs/SetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlMode-request>)))
  "Returns md5sum for a message object of type '<SetControlMode-request>"
  "f6dd4c28fb851ba56deb138366053217")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlMode-request)))
  "Returns md5sum for a message object of type 'SetControlMode-request"
  "f6dd4c28fb851ba56deb138366053217")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlMode-request>)))
  "Returns full string definition for message of type '<SetControlMode-request>"
  (cl:format cl:nil "# request~%int32 id~%int32 max_repeats~%string control~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlMode-request)))
  "Returns full string definition for message of type 'SetControlMode-request"
  (cl:format cl:nil "# request~%int32 id~%int32 max_repeats~%string control~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlMode-request>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'control))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlMode-request
    (cl:cons ':id (id msg))
    (cl:cons ':max_repeats (max_repeats msg))
    (cl:cons ':control (control msg))
))
;//! \htmlinclude SetControlMode-response.msg.html

(cl:defclass <SetControlMode-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetControlMode-response (<SetControlMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<SetControlMode-response> is deprecated: use qb_device_srvs-srv:SetControlMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetControlMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:success-val is deprecated.  Use qb_device_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'failures-val :lambda-list '(m))
(cl:defmethod failures-val ((m <SetControlMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:failures-val is deprecated.  Use qb_device_srvs-srv:failures instead.")
  (failures m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlMode-response>) ostream)
  "Serializes a message object of type '<SetControlMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'failures)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlMode-response>) istream)
  "Deserializes a message object of type '<SetControlMode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'failures) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlMode-response>)))
  "Returns string type for a service object of type '<SetControlMode-response>"
  "qb_device_srvs/SetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode-response)))
  "Returns string type for a service object of type 'SetControlMode-response"
  "qb_device_srvs/SetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlMode-response>)))
  "Returns md5sum for a message object of type '<SetControlMode-response>"
  "f6dd4c28fb851ba56deb138366053217")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlMode-response)))
  "Returns md5sum for a message object of type 'SetControlMode-response"
  "f6dd4c28fb851ba56deb138366053217")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlMode-response>)))
  "Returns full string definition for message of type '<SetControlMode-response>"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlMode-response)))
  "Returns full string definition for message of type 'SetControlMode-response"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlMode-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlMode-response
    (cl:cons ':success (success msg))
    (cl:cons ':failures (failures msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetControlMode)))
  'SetControlMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetControlMode)))
  'SetControlMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode)))
  "Returns string type for a service object of type '<SetControlMode>"
  "qb_device_srvs/SetControlMode")