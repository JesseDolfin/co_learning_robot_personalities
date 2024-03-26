;; Auto-generated. Do not edit!


(when (boundp 'qb_device_msgs::ConnectionState)
  (if (not (find-package "QB_DEVICE_MSGS"))
    (make-package "QB_DEVICE_MSGS"))
  (shadow 'ConnectionState (find-package "QB_DEVICE_MSGS")))
(unless (find-package "QB_DEVICE_MSGS::CONNECTIONSTATE")
  (make-package "QB_DEVICE_MSGS::CONNECTIONSTATE"))

(in-package "ROS")
;;//! \htmlinclude ConnectionState.msg.html


(defclass qb_device_msgs::ConnectionState
  :super ros::object
  :slots (_devices ))

(defmethod qb_device_msgs::ConnectionState
  (:init
   (&key
    ((:devices __devices) ())
    )
   (send-super :init)
   (setq _devices __devices)
   self)
  (:devices
   (&rest __devices)
   (if (keywordp (car __devices))
       (send* _devices __devices)
     (progn
       (if __devices (setq _devices (car __devices)))
       _devices)))
  (:serialization-length
   ()
   (+
    ;; qb_device_msgs/DeviceConnectionInfo[] _devices
    (apply #'+ (send-all _devices :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; qb_device_msgs/DeviceConnectionInfo[] _devices
     (write-long (length _devices) s)
     (dolist (elem _devices)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; qb_device_msgs/DeviceConnectionInfo[] _devices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _devices (let (r) (dotimes (i n) (push (instance qb_device_msgs::DeviceConnectionInfo :init) r)) r))
     (dolist (elem- _devices)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get qb_device_msgs::ConnectionState :md5sum-) "3fb8aec8a16b727f8528545cb7e04895")
(setf (get qb_device_msgs::ConnectionState :datatype-) "qb_device_msgs/ConnectionState")
(setf (get qb_device_msgs::ConnectionState :definition-)
      "# Device-independent connection state message

qb_device_msgs/DeviceConnectionInfo[] devices
================================================================================
MSG: qb_device_msgs/DeviceConnectionInfo
# Device-independent message that constains: 

int32 id           # device id;
bool is_active     # motor activation status;
string port        # serial port
")



(provide :qb_device_msgs/ConnectionState "3fb8aec8a16b727f8528545cb7e04895")


