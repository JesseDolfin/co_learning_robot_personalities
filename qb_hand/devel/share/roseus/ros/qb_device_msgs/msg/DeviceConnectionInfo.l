;; Auto-generated. Do not edit!


(when (boundp 'qb_device_msgs::DeviceConnectionInfo)
  (if (not (find-package "QB_DEVICE_MSGS"))
    (make-package "QB_DEVICE_MSGS"))
  (shadow 'DeviceConnectionInfo (find-package "QB_DEVICE_MSGS")))
(unless (find-package "QB_DEVICE_MSGS::DEVICECONNECTIONINFO")
  (make-package "QB_DEVICE_MSGS::DEVICECONNECTIONINFO"))

(in-package "ROS")
;;//! \htmlinclude DeviceConnectionInfo.msg.html


(defclass qb_device_msgs::DeviceConnectionInfo
  :super ros::object
  :slots (_id _is_active _port ))

(defmethod qb_device_msgs::DeviceConnectionInfo
  (:init
   (&key
    ((:id __id) 0)
    ((:is_active __is_active) nil)
    ((:port __port) "")
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _is_active __is_active)
   (setq _port (string __port))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:is_active
   (&optional (__is_active :null))
   (if (not (eq __is_active :null)) (setq _is_active __is_active)) _is_active)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; bool _is_active
    1
    ;; string _port
    4 (length _port)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; bool _is_active
       (if _is_active (write-byte -1 s) (write-byte 0 s))
     ;; string _port
       (write-long (length _port) s) (princ _port s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _is_active
     (setq _is_active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _port
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _port (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get qb_device_msgs::DeviceConnectionInfo :md5sum-) "1a1c593244281b064cf77ab64e673fe3")
(setf (get qb_device_msgs::DeviceConnectionInfo :datatype-) "qb_device_msgs/DeviceConnectionInfo")
(setf (get qb_device_msgs::DeviceConnectionInfo :definition-)
      "# Device-independent message that constains: 

int32 id           # device id;
bool is_active     # motor activation status;
string port        # serial port
")



(provide :qb_device_msgs/DeviceConnectionInfo "1a1c593244281b064cf77ab64e673fe3")


