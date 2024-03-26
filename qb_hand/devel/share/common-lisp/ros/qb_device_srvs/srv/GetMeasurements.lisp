; Auto-generated. Do not edit!


(cl:in-package qb_device_srvs-srv)


;//! \htmlinclude GetMeasurements-request.msg.html

(cl:defclass <GetMeasurements-request> (roslisp-msg-protocol:ros-message)
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
   (get_positions
    :reader get_positions
    :initarg :get_positions
    :type cl:boolean
    :initform cl:nil)
   (get_currents
    :reader get_currents
    :initarg :get_currents
    :type cl:boolean
    :initform cl:nil)
   (get_distinct_packages
    :reader get_distinct_packages
    :initarg :get_distinct_packages
    :type cl:boolean
    :initform cl:nil)
   (get_commands
    :reader get_commands
    :initarg :get_commands
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetMeasurements-request (<GetMeasurements-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMeasurements-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMeasurements-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<GetMeasurements-request> is deprecated: use qb_device_srvs-srv:GetMeasurements-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <GetMeasurements-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:id-val is deprecated.  Use qb_device_srvs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'max_repeats-val :lambda-list '(m))
(cl:defmethod max_repeats-val ((m <GetMeasurements-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:max_repeats-val is deprecated.  Use qb_device_srvs-srv:max_repeats instead.")
  (max_repeats m))

(cl:ensure-generic-function 'get_positions-val :lambda-list '(m))
(cl:defmethod get_positions-val ((m <GetMeasurements-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:get_positions-val is deprecated.  Use qb_device_srvs-srv:get_positions instead.")
  (get_positions m))

(cl:ensure-generic-function 'get_currents-val :lambda-list '(m))
(cl:defmethod get_currents-val ((m <GetMeasurements-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:get_currents-val is deprecated.  Use qb_device_srvs-srv:get_currents instead.")
  (get_currents m))

(cl:ensure-generic-function 'get_distinct_packages-val :lambda-list '(m))
(cl:defmethod get_distinct_packages-val ((m <GetMeasurements-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:get_distinct_packages-val is deprecated.  Use qb_device_srvs-srv:get_distinct_packages instead.")
  (get_distinct_packages m))

(cl:ensure-generic-function 'get_commands-val :lambda-list '(m))
(cl:defmethod get_commands-val ((m <GetMeasurements-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:get_commands-val is deprecated.  Use qb_device_srvs-srv:get_commands instead.")
  (get_commands m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMeasurements-request>) ostream)
  "Serializes a message object of type '<GetMeasurements-request>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'get_positions) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'get_currents) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'get_distinct_packages) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'get_commands) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMeasurements-request>) istream)
  "Deserializes a message object of type '<GetMeasurements-request>"
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
    (cl:setf (cl:slot-value msg 'get_positions) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'get_currents) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'get_distinct_packages) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'get_commands) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMeasurements-request>)))
  "Returns string type for a service object of type '<GetMeasurements-request>"
  "qb_device_srvs/GetMeasurementsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasurements-request)))
  "Returns string type for a service object of type 'GetMeasurements-request"
  "qb_device_srvs/GetMeasurementsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMeasurements-request>)))
  "Returns md5sum for a message object of type '<GetMeasurements-request>"
  "61d005acb1e04c16b9b33b19436d5ede")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMeasurements-request)))
  "Returns md5sum for a message object of type 'GetMeasurements-request"
  "61d005acb1e04c16b9b33b19436d5ede")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMeasurements-request>)))
  "Returns full string definition for message of type '<GetMeasurements-request>"
  (cl:format cl:nil "# request~%int32 id~%int32 max_repeats~%bool get_positions~%bool get_currents~%bool get_distinct_packages~%bool get_commands~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMeasurements-request)))
  "Returns full string definition for message of type 'GetMeasurements-request"
  (cl:format cl:nil "# request~%int32 id~%int32 max_repeats~%bool get_positions~%bool get_currents~%bool get_distinct_packages~%bool get_commands~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMeasurements-request>))
  (cl:+ 0
     4
     4
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMeasurements-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMeasurements-request
    (cl:cons ':id (id msg))
    (cl:cons ':max_repeats (max_repeats msg))
    (cl:cons ':get_positions (get_positions msg))
    (cl:cons ':get_currents (get_currents msg))
    (cl:cons ':get_distinct_packages (get_distinct_packages msg))
    (cl:cons ':get_commands (get_commands msg))
))
;//! \htmlinclude GetMeasurements-response.msg.html

(cl:defclass <GetMeasurements-response> (roslisp-msg-protocol:ros-message)
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
   (positions
    :reader positions
    :initarg :positions
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (currents
    :reader currents
    :initarg :currents
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (commands
    :reader commands
    :initarg :commands
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0))
)

(cl:defclass GetMeasurements-response (<GetMeasurements-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMeasurements-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMeasurements-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qb_device_srvs-srv:<GetMeasurements-response> is deprecated: use qb_device_srvs-srv:GetMeasurements-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetMeasurements-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:success-val is deprecated.  Use qb_device_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'failures-val :lambda-list '(m))
(cl:defmethod failures-val ((m <GetMeasurements-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:failures-val is deprecated.  Use qb_device_srvs-srv:failures instead.")
  (failures m))

(cl:ensure-generic-function 'positions-val :lambda-list '(m))
(cl:defmethod positions-val ((m <GetMeasurements-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:positions-val is deprecated.  Use qb_device_srvs-srv:positions instead.")
  (positions m))

(cl:ensure-generic-function 'currents-val :lambda-list '(m))
(cl:defmethod currents-val ((m <GetMeasurements-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:currents-val is deprecated.  Use qb_device_srvs-srv:currents instead.")
  (currents m))

(cl:ensure-generic-function 'commands-val :lambda-list '(m))
(cl:defmethod commands-val ((m <GetMeasurements-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:commands-val is deprecated.  Use qb_device_srvs-srv:commands instead.")
  (commands m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <GetMeasurements-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qb_device_srvs-srv:stamp-val is deprecated.  Use qb_device_srvs-srv:stamp instead.")
  (stamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMeasurements-response>) ostream)
  "Serializes a message object of type '<GetMeasurements-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'failures)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'positions))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'currents))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'currents))
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
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMeasurements-response>) istream)
  "Deserializes a message object of type '<GetMeasurements-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'failures) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'positions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'positions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'currents) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'currents)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
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
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMeasurements-response>)))
  "Returns string type for a service object of type '<GetMeasurements-response>"
  "qb_device_srvs/GetMeasurementsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasurements-response)))
  "Returns string type for a service object of type 'GetMeasurements-response"
  "qb_device_srvs/GetMeasurementsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMeasurements-response>)))
  "Returns md5sum for a message object of type '<GetMeasurements-response>"
  "61d005acb1e04c16b9b33b19436d5ede")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMeasurements-response)))
  "Returns md5sum for a message object of type 'GetMeasurements-response"
  "61d005acb1e04c16b9b33b19436d5ede")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMeasurements-response>)))
  "Returns full string definition for message of type '<GetMeasurements-response>"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%int16[] positions~%int16[] currents~%int16[] commands~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMeasurements-response)))
  "Returns full string definition for message of type 'GetMeasurements-response"
  (cl:format cl:nil "# response~%bool success~%int32 failures~%int16[] positions~%int16[] currents~%int16[] commands~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMeasurements-response>))
  (cl:+ 0
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'currents) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'commands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMeasurements-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMeasurements-response
    (cl:cons ':success (success msg))
    (cl:cons ':failures (failures msg))
    (cl:cons ':positions (positions msg))
    (cl:cons ':currents (currents msg))
    (cl:cons ':commands (commands msg))
    (cl:cons ':stamp (stamp msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMeasurements)))
  'GetMeasurements-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMeasurements)))
  'GetMeasurements-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasurements)))
  "Returns string type for a service object of type '<GetMeasurements>"
  "qb_device_srvs/GetMeasurements")