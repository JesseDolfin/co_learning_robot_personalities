;; Auto-generated. Do not edit!


(when (boundp 'qb_device_msgs::State)
  (if (not (find-package "QB_DEVICE_MSGS"))
    (make-package "QB_DEVICE_MSGS"))
  (shadow 'State (find-package "QB_DEVICE_MSGS")))
(unless (find-package "QB_DEVICE_MSGS::STATE")
  (make-package "QB_DEVICE_MSGS::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass qb_device_msgs::State
  :super ros::object
  :slots (_actuators _joints _is_reliable _consecutive_failures ))

(defmethod qb_device_msgs::State
  (:init
   (&key
    ((:actuators __actuators) ())
    ((:joints __joints) ())
    ((:is_reliable __is_reliable) nil)
    ((:consecutive_failures __consecutive_failures) 0)
    )
   (send-super :init)
   (setq _actuators __actuators)
   (setq _joints __joints)
   (setq _is_reliable __is_reliable)
   (setq _consecutive_failures (round __consecutive_failures))
   self)
  (:actuators
   (&rest __actuators)
   (if (keywordp (car __actuators))
       (send* _actuators __actuators)
     (progn
       (if __actuators (setq _actuators (car __actuators)))
       _actuators)))
  (:joints
   (&rest __joints)
   (if (keywordp (car __joints))
       (send* _joints __joints)
     (progn
       (if __joints (setq _joints (car __joints)))
       _joints)))
  (:is_reliable
   (&optional (__is_reliable :null))
   (if (not (eq __is_reliable :null)) (setq _is_reliable __is_reliable)) _is_reliable)
  (:consecutive_failures
   (&optional __consecutive_failures)
   (if __consecutive_failures (setq _consecutive_failures __consecutive_failures)) _consecutive_failures)
  (:serialization-length
   ()
   (+
    ;; qb_device_msgs/ResourceData[] _actuators
    (apply #'+ (send-all _actuators :serialization-length)) 4
    ;; qb_device_msgs/ResourceData[] _joints
    (apply #'+ (send-all _joints :serialization-length)) 4
    ;; bool _is_reliable
    1
    ;; int32 _consecutive_failures
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; qb_device_msgs/ResourceData[] _actuators
     (write-long (length _actuators) s)
     (dolist (elem _actuators)
       (send elem :serialize s)
       )
     ;; qb_device_msgs/ResourceData[] _joints
     (write-long (length _joints) s)
     (dolist (elem _joints)
       (send elem :serialize s)
       )
     ;; bool _is_reliable
       (if _is_reliable (write-byte -1 s) (write-byte 0 s))
     ;; int32 _consecutive_failures
       (write-long _consecutive_failures s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; qb_device_msgs/ResourceData[] _actuators
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _actuators (let (r) (dotimes (i n) (push (instance qb_device_msgs::ResourceData :init) r)) r))
     (dolist (elem- _actuators)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; qb_device_msgs/ResourceData[] _joints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joints (let (r) (dotimes (i n) (push (instance qb_device_msgs::ResourceData :init) r)) r))
     (dolist (elem- _joints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _is_reliable
     (setq _is_reliable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _consecutive_failures
     (setq _consecutive_failures (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get qb_device_msgs::State :md5sum-) "035992012f0af1c782c17a0f8f6e544c")
(setf (get qb_device_msgs::State :datatype-) "qb_device_msgs/State")
(setf (get qb_device_msgs::State :definition-)
      "# State message valid for either qbhand or qbmove

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



(provide :qb_device_msgs/State "035992012f0af1c782c17a0f8f6e544c")


