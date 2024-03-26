;; Auto-generated. Do not edit!


(when (boundp 'qb_device_srvs::InitializeDevice)
  (if (not (find-package "QB_DEVICE_SRVS"))
    (make-package "QB_DEVICE_SRVS"))
  (shadow 'InitializeDevice (find-package "QB_DEVICE_SRVS")))
(unless (find-package "QB_DEVICE_SRVS::INITIALIZEDEVICE")
  (make-package "QB_DEVICE_SRVS::INITIALIZEDEVICE"))
(unless (find-package "QB_DEVICE_SRVS::INITIALIZEDEVICEREQUEST")
  (make-package "QB_DEVICE_SRVS::INITIALIZEDEVICEREQUEST"))
(unless (find-package "QB_DEVICE_SRVS::INITIALIZEDEVICERESPONSE")
  (make-package "QB_DEVICE_SRVS::INITIALIZEDEVICERESPONSE"))

(in-package "ROS")



(if (not (find-package "QB_DEVICE_MSGS"))
  (ros::roseus-add-msgs "qb_device_msgs"))


(defclass qb_device_srvs::InitializeDeviceRequest
  :super ros::object
  :slots (_id _activate _rescan _max_repeats ))

(defmethod qb_device_srvs::InitializeDeviceRequest
  (:init
   (&key
    ((:id __id) 0)
    ((:activate __activate) nil)
    ((:rescan __rescan) nil)
    ((:max_repeats __max_repeats) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _activate __activate)
   (setq _rescan __rescan)
   (setq _max_repeats (round __max_repeats))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:activate
   (&optional (__activate :null))
   (if (not (eq __activate :null)) (setq _activate __activate)) _activate)
  (:rescan
   (&optional (__rescan :null))
   (if (not (eq __rescan :null)) (setq _rescan __rescan)) _rescan)
  (:max_repeats
   (&optional __max_repeats)
   (if __max_repeats (setq _max_repeats __max_repeats)) _max_repeats)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; bool _activate
    1
    ;; bool _rescan
    1
    ;; int32 _max_repeats
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; bool _activate
       (if _activate (write-byte -1 s) (write-byte 0 s))
     ;; bool _rescan
       (if _rescan (write-byte -1 s) (write-byte 0 s))
     ;; int32 _max_repeats
       (write-long _max_repeats s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _activate
     (setq _activate (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _rescan
     (setq _rescan (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _max_repeats
     (setq _max_repeats (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass qb_device_srvs::InitializeDeviceResponse
  :super ros::object
  :slots (_success _failures _message _info ))

(defmethod qb_device_srvs::InitializeDeviceResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:failures __failures) 0)
    ((:message __message) "")
    ((:info __info) (instance qb_device_msgs::Info :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _failures (round __failures))
   (setq _message (string __message))
   (setq _info __info)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:failures
   (&optional __failures)
   (if __failures (setq _failures __failures)) _failures)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:info
   (&rest __info)
   (if (keywordp (car __info))
       (send* _info __info)
     (progn
       (if __info (setq _info (car __info)))
       _info)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _failures
    4
    ;; string _message
    4 (length _message)
    ;; qb_device_msgs/Info _info
    (send _info :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _failures
       (write-long _failures s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; qb_device_msgs/Info _info
       (send _info :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _failures
     (setq _failures (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; qb_device_msgs/Info _info
     (send _info :deserialize buf ptr-) (incf ptr- (send _info :serialization-length))
   ;;
   self)
  )

(defclass qb_device_srvs::InitializeDevice
  :super ros::object
  :slots ())

(setf (get qb_device_srvs::InitializeDevice :md5sum-) "c23409117bb3e9fa24111fdf884ca1f3")
(setf (get qb_device_srvs::InitializeDevice :datatype-) "qb_device_srvs/InitializeDevice")
(setf (get qb_device_srvs::InitializeDevice :request) qb_device_srvs::InitializeDeviceRequest)
(setf (get qb_device_srvs::InitializeDevice :response) qb_device_srvs::InitializeDeviceResponse)

(defmethod qb_device_srvs::InitializeDeviceRequest
  (:response () (instance qb_device_srvs::InitializeDeviceResponse :init)))

(setf (get qb_device_srvs::InitializeDeviceRequest :md5sum-) "c23409117bb3e9fa24111fdf884ca1f3")
(setf (get qb_device_srvs::InitializeDeviceRequest :datatype-) "qb_device_srvs/InitializeDeviceRequest")
(setf (get qb_device_srvs::InitializeDeviceRequest :definition-)
      "# request
int32 id
bool activate
bool rescan
int32 max_repeats
---
# response
bool success
int32 failures
string message
qb_device_msgs/Info info

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
")

(setf (get qb_device_srvs::InitializeDeviceResponse :md5sum-) "c23409117bb3e9fa24111fdf884ca1f3")
(setf (get qb_device_srvs::InitializeDeviceResponse :datatype-) "qb_device_srvs/InitializeDeviceResponse")
(setf (get qb_device_srvs::InitializeDeviceResponse :definition-)
      "# request
int32 id
bool activate
bool rescan
int32 max_repeats
---
# response
bool success
int32 failures
string message
qb_device_msgs/Info info

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
")



(provide :qb_device_srvs/InitializeDevice "c23409117bb3e9fa24111fdf884ca1f3")


