;; Auto-generated. Do not edit!


(when (boundp 'qb_device_srvs::SetControlMode)
  (if (not (find-package "QB_DEVICE_SRVS"))
    (make-package "QB_DEVICE_SRVS"))
  (shadow 'SetControlMode (find-package "QB_DEVICE_SRVS")))
(unless (find-package "QB_DEVICE_SRVS::SETCONTROLMODE")
  (make-package "QB_DEVICE_SRVS::SETCONTROLMODE"))
(unless (find-package "QB_DEVICE_SRVS::SETCONTROLMODEREQUEST")
  (make-package "QB_DEVICE_SRVS::SETCONTROLMODEREQUEST"))
(unless (find-package "QB_DEVICE_SRVS::SETCONTROLMODERESPONSE")
  (make-package "QB_DEVICE_SRVS::SETCONTROLMODERESPONSE"))

(in-package "ROS")





(defclass qb_device_srvs::SetControlModeRequest
  :super ros::object
  :slots (_id _max_repeats _control ))

(defmethod qb_device_srvs::SetControlModeRequest
  (:init
   (&key
    ((:id __id) 0)
    ((:max_repeats __max_repeats) 0)
    ((:control __control) "")
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _max_repeats (round __max_repeats))
   (setq _control (string __control))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:max_repeats
   (&optional __max_repeats)
   (if __max_repeats (setq _max_repeats __max_repeats)) _max_repeats)
  (:control
   (&optional __control)
   (if __control (setq _control __control)) _control)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _max_repeats
    4
    ;; string _control
    4 (length _control)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _max_repeats
       (write-long _max_repeats s)
     ;; string _control
       (write-long (length _control) s) (princ _control s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_repeats
     (setq _max_repeats (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _control
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _control (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass qb_device_srvs::SetControlModeResponse
  :super ros::object
  :slots (_success _failures ))

(defmethod qb_device_srvs::SetControlModeResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:failures __failures) 0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _failures (round __failures))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:failures
   (&optional __failures)
   (if __failures (setq _failures __failures)) _failures)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _failures
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _failures
       (write-long _failures s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _failures
     (setq _failures (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass qb_device_srvs::SetControlMode
  :super ros::object
  :slots ())

(setf (get qb_device_srvs::SetControlMode :md5sum-) "f6dd4c28fb851ba56deb138366053217")
(setf (get qb_device_srvs::SetControlMode :datatype-) "qb_device_srvs/SetControlMode")
(setf (get qb_device_srvs::SetControlMode :request) qb_device_srvs::SetControlModeRequest)
(setf (get qb_device_srvs::SetControlMode :response) qb_device_srvs::SetControlModeResponse)

(defmethod qb_device_srvs::SetControlModeRequest
  (:response () (instance qb_device_srvs::SetControlModeResponse :init)))

(setf (get qb_device_srvs::SetControlModeRequest :md5sum-) "f6dd4c28fb851ba56deb138366053217")
(setf (get qb_device_srvs::SetControlModeRequest :datatype-) "qb_device_srvs/SetControlModeRequest")
(setf (get qb_device_srvs::SetControlModeRequest :definition-)
      "# request
int32 id
int32 max_repeats
string control
---
# response
bool success
int32 failures
")

(setf (get qb_device_srvs::SetControlModeResponse :md5sum-) "f6dd4c28fb851ba56deb138366053217")
(setf (get qb_device_srvs::SetControlModeResponse :datatype-) "qb_device_srvs/SetControlModeResponse")
(setf (get qb_device_srvs::SetControlModeResponse :definition-)
      "# request
int32 id
int32 max_repeats
string control
---
# response
bool success
int32 failures
")



(provide :qb_device_srvs/SetControlMode "f6dd4c28fb851ba56deb138366053217")


