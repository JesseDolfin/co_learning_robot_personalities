;; Auto-generated. Do not edit!


(when (boundp 'qb_device_srvs::Trigger)
  (if (not (find-package "QB_DEVICE_SRVS"))
    (make-package "QB_DEVICE_SRVS"))
  (shadow 'Trigger (find-package "QB_DEVICE_SRVS")))
(unless (find-package "QB_DEVICE_SRVS::TRIGGER")
  (make-package "QB_DEVICE_SRVS::TRIGGER"))
(unless (find-package "QB_DEVICE_SRVS::TRIGGERREQUEST")
  (make-package "QB_DEVICE_SRVS::TRIGGERREQUEST"))
(unless (find-package "QB_DEVICE_SRVS::TRIGGERRESPONSE")
  (make-package "QB_DEVICE_SRVS::TRIGGERRESPONSE"))

(in-package "ROS")





(defclass qb_device_srvs::TriggerRequest
  :super ros::object
  :slots (_id _max_repeats ))

(defmethod qb_device_srvs::TriggerRequest
  (:init
   (&key
    ((:id __id) 0)
    ((:max_repeats __max_repeats) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _max_repeats (round __max_repeats))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:max_repeats
   (&optional __max_repeats)
   (if __max_repeats (setq _max_repeats __max_repeats)) _max_repeats)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _max_repeats
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _max_repeats
       (write-long _max_repeats s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_repeats
     (setq _max_repeats (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass qb_device_srvs::TriggerResponse
  :super ros::object
  :slots (_success _failures _message ))

(defmethod qb_device_srvs::TriggerResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:failures __failures) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _failures (round __failures))
   (setq _message (string __message))
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
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _failures
    4
    ;; string _message
    4 (length _message)
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
   ;;
   self)
  )

(defclass qb_device_srvs::Trigger
  :super ros::object
  :slots ())

(setf (get qb_device_srvs::Trigger :md5sum-) "d696ad9232824524315696c7565a79d7")
(setf (get qb_device_srvs::Trigger :datatype-) "qb_device_srvs/Trigger")
(setf (get qb_device_srvs::Trigger :request) qb_device_srvs::TriggerRequest)
(setf (get qb_device_srvs::Trigger :response) qb_device_srvs::TriggerResponse)

(defmethod qb_device_srvs::TriggerRequest
  (:response () (instance qb_device_srvs::TriggerResponse :init)))

(setf (get qb_device_srvs::TriggerRequest :md5sum-) "d696ad9232824524315696c7565a79d7")
(setf (get qb_device_srvs::TriggerRequest :datatype-) "qb_device_srvs/TriggerRequest")
(setf (get qb_device_srvs::TriggerRequest :definition-)
      "# request
int32 id
int32 max_repeats
---
# response
bool success
int32 failures
string message
")

(setf (get qb_device_srvs::TriggerResponse :md5sum-) "d696ad9232824524315696c7565a79d7")
(setf (get qb_device_srvs::TriggerResponse :datatype-) "qb_device_srvs/TriggerResponse")
(setf (get qb_device_srvs::TriggerResponse :definition-)
      "# request
int32 id
int32 max_repeats
---
# response
bool success
int32 failures
string message
")



(provide :qb_device_srvs/Trigger "d696ad9232824524315696c7565a79d7")


