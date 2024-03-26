;; Auto-generated. Do not edit!


(when (boundp 'qb_device_msgs::StateStamped)
  (if (not (find-package "QB_DEVICE_MSGS"))
    (make-package "QB_DEVICE_MSGS"))
  (shadow 'StateStamped (find-package "QB_DEVICE_MSGS")))
(unless (find-package "QB_DEVICE_MSGS::STATESTAMPED")
  (make-package "QB_DEVICE_MSGS::STATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude StateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass qb_device_msgs::StateStamped
  :super ros::object
  :slots (_header _device_info _device_data ))

(defmethod qb_device_msgs::StateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:device_info __device_info) (instance qb_device_msgs::Info :init))
    ((:device_data __device_data) (instance qb_device_msgs::State :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _device_info __device_info)
   (setq _device_data __device_data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:device_info
   (&rest __device_info)
   (if (keywordp (car __device_info))
       (send* _device_info __device_info)
     (progn
       (if __device_info (setq _device_info (car __device_info)))
       _device_info)))
  (:device_data
   (&rest __device_data)
   (if (keywordp (car __device_data))
       (send* _device_data __device_data)
     (progn
       (if __device_data (setq _device_data (car __device_data)))
       _device_data)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; qb_device_msgs/Info _device_info
    (send _device_info :serialization-length)
    ;; qb_device_msgs/State _device_data
    (send _device_data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; qb_device_msgs/Info _device_info
       (send _device_info :serialize s)
     ;; qb_device_msgs/State _device_data
       (send _device_data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; qb_device_msgs/Info _device_info
     (send _device_info :deserialize buf ptr-) (incf ptr- (send _device_info :serialization-length))
   ;; qb_device_msgs/State _device_data
     (send _device_data :deserialize buf ptr-) (incf ptr- (send _device_data :serialization-length))
   ;;
   self)
  )

(setf (get qb_device_msgs::StateStamped :md5sum-) "4ad56bd88424f6cfda763bbf1b38cce8")
(setf (get qb_device_msgs::StateStamped :datatype-) "qb_device_msgs/StateStamped")
(setf (get qb_device_msgs::StateStamped :definition-)
      "# State message with stamped info valid for either qbhand or qbmove

std_msgs/Header header

qb_device_msgs/Info device_info

qb_device_msgs/State device_data
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: qb_device_msgs/Info
# Standard device-independent info message

int32 id
string serial_port
int32 max_repeats
bool get_positions
bool get_currents
bool get_distinct_packages
bool set_commands
bool set_commands_async
int32[] position_limits
uint8[] encoder_resolutions
================================================================================
MSG: qb_device_msgs/State
# State message valid for either qbhand or qbmove

# either qbhand or qbmove:
#  - motors: position, command in [ticks], velocity in [ticks/s], effort in [mA]
qb_device_msgs/ResourceData[] actuators

# qbhand:
#  - closure: position, command in [0,1], velocity in [percent/s],  effort in [A].
# qbmove:
#  - shaft: position, command in [radians], velocity in [radians/s], effort in [A];
#  - preset: position, command in [0,1], velocity in [percent/s], effort is not used.
qb_device_msgs/ResourceData[] joints

# Reliability of the retrieved measurements
bool is_reliable
int32 consecutive_failures
================================================================================
MSG: qb_device_msgs/ResourceData
# Device-independent resource data message

string name
float64 position
float64 velocity
float64 effort
float64 command
")



(provide :qb_device_msgs/StateStamped "4ad56bd88424f6cfda763bbf1b38cce8")


