; Auto-generated. Do not edit!


(cl:in-package qb_device_srvs-srv)


;//! \htmlinclude SetPID-request.msg.html

(cl:defclass <SetPID-request> (roslisp-msg-protocol:ros-message)
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
   (p
    :reader p
    :initarg :p
    :type cl:float
    :initform 0.0)
   (i
    :reader i
    :initarg :i
    :type cl:float
    :initform 0.0)
   (d
    :reader d
    :initarg :d
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetPID-request (<SetPID-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPID-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPID-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<SetPID-request> is deprecated: use qb_device_srvs-srv:SetPID-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:id-val is deprecated.  Use qb_device_srvs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'max_repeats-val :lambda-list '(m))
(cl:defmethod max_repeats-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:max_repeats-val is deprecated.  Use qb_device_srvs-srv:max_repeats instead.")
  (max_repeats m))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:p-val is deprecated.  Use qb_device_srvs-srv:p instead.")
  (p m))

(cl:ensure-generic-function 'i-val :lambda-list '(m))
(cl:defmethod i-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:i-val is deprecated.  Use qb_device_srvs-srv:i instead.")
  (i m))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <SetPID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:d-val is deprecated.  Use qb_device_srvs-srv:d instead.")
  (d m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPID-request>) ostream)
  "Serializes a message object of type '<SetPID-request>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'i))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPID-request>) istream)
  "Deserializes a message object of type '<SetPID-request>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'i) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPID-request>)))
  "Returns string type for a service object of type '<SetPID-request>"
  "qb_device_srvs/SetPIDRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPID-request)))
  "Returns string type for a service object of type 'SetPID-request"
  "qb_device_srvs/SetPIDRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPID-request>)))
  "Returns md5sum for a message object of type '<SetPID-request>"
  "924f2bcd17c4f9d95c42467a96352ac1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPID-request)))
  "Returns md5sum for a message object of type 'SetPID-request"
  "924f2bcd17c4f9d95c42467a96352ac1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPID-request>)))
  "Returns full string definition for message of type '<SetPID-request>"
  (cl:format cl:nil "# request~%int32 id~%int32 max_repeats~%float32 p~%float32 i~%float32 d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPID-request)))
  "Returns full string definition for message of type 'SetPID-request"
  (cl:format cl:nil "# request~%int32 id~%int32 max_repeats~%float32 p~%float32 i~%float32 d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPID-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPID-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPID-request
    (cl:cons ':id (id msg))
    (cl:cons ':max_repeats (max_repeats msg))
    (cl:cons ':p (p msg))
    (cl:cons ':i (i msg))
    (cl:cons ':d (d msg))
))
;//! \htmlinclude SetPID-response.msg.html

(cl:defclass <SetPID-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetPID-response (<SetPID-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPID-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPID-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<SetPID-response> is deprecated: use qb_device_srvs-srv:SetPID-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetPID-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:success-val is deprecated.  Use qb_device_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'failures-val :lambda-list '(m))
(cl:defmethod failures-val ((m <SetPID-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:failures-val is deprecated.  Use qb_device_srvs-srv:failures instead.")
  (failures m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPID-response>) ostream)
  "Serializes a message object of type '<SetPID-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'failures)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPID-response>) istream)
  "Deserializes a message object of type '<SetPID-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'failures) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPID-response>)))
  "Returns string type for a service object of type '<SetPID-response>"
  "qb_device_srvs/SetPIDResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPID-response)))
  "Returns string type for a service object of type 'SetPID-response"
  "qb_device_srvs/SetPIDResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPID-response>)))
  "Returns md5sum for a message object of type '<SetPID-response>"
  "924f2bcd17c4f9d95c42467a96352ac1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPID-response)))
  "Returns md5sum for a message object of type 'SetPID-response"
  "924f2bcd17c4f9d95c42467a96352ac1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPID-response>)))
  "Returns full string definition for message of type '<SetPID-response>"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPID-response)))
  "Returns full string definition for message of type 'SetPID-response"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPID-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPID-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPID-response
    (cl:cons ':success (success msg))
    (cl:cons ':failures (failures msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPID)))
  'SetPID-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPID)))
  'SetPID-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPID)))
  "Returns string type for a service object of type '<SetPID>"
  "qb_device_srvs/SetPID")