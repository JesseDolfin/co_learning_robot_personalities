;; Auto-generated. Do not edit!


(when (boundp 'qb_device_srvs::SetPID)
  (if (not (find-package "QB_DEVICE_SRVS"))
    (make-package "QB_DEVICE_SRVS"))
  (shadow 'SetPID (find-package "QB_DEVICE_SRVS")))
(unless (find-package "QB_DEVICE_SRVS::SETPID")
  (make-package "QB_DEVICE_SRVS::SETPID"))
(unless (find-package "QB_DEVICE_SRVS::SETPIDREQUEST")
  (make-package "QB_DEVICE_SRVS::SETPIDREQUEST"))
(unless (find-package "QB_DEVICE_SRVS::SETPIDRESPONSE")
  (make-package "QB_DEVICE_SRVS::SETPIDRESPONSE"))

(in-package "ROS")





(defclass qb_device_srvs::SetPIDRequest
  :super ros::object
  :slots (_id _max_repeats _p _i _d ))

(defmethod qb_device_srvs::SetPIDRequest
  (:init
   (&key
    ((:id __id) 0)
    ((:max_repeats __max_repeats) 0)
    ((:p __p) 0.0)
    ((:i __i) 0.0)
    ((:d __d) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _max_repeats (round __max_repeats))
   (setq _p (float __p))
   (setq _i (float __i))
   (setq _d (float __d))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:max_repeats
   (&optional __max_repeats)
   (if __max_repeats (setq _max_repeats __max_repeats)) _max_repeats)
  (:p
   (&optional __p)
   (if __p (setq _p __p)) _p)
  (:i
   (&optional __i)
   (if __i (setq _i __i)) _i)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _max_repeats
    4
    ;; float32 _p
    4
    ;; float32 _i
    4
    ;; float32 _d
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
     ;; float32 _p
       (sys::poke _p (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _i
       (sys::poke _i (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _d
       (sys::poke _d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_repeats
     (setq _max_repeats (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _p
     (setq _p (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _i
     (setq _i (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _d
     (setq _d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass qb_device_srvs::SetPIDResponse
  :super ros::object
  :slots (_success _failures ))

(defmethod qb_device_srvs::SetPIDResponse
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

(defclass qb_device_srvs::SetPID
  :super ros::object
  :slots ())

(setf (get qb_device_srvs::SetPID :md5sum-) "924f2bcd17c4f9d95c42467a96352ac1")
(setf (get qb_device_srvs::SetPID :datatype-) "qb_device_srvs/SetPID")
(setf (get qb_device_srvs::SetPID :request) qb_device_srvs::SetPIDRequest)
(setf (get qb_device_srvs::SetPID :response) qb_device_srvs::SetPIDResponse)

(defmethod qb_device_srvs::SetPIDRequest
  (:response () (instance qb_device_srvs::SetPIDResponse :init)))

(setf (get qb_device_srvs::SetPIDRequest :md5sum-) "924f2bcd17c4f9d95c42467a96352ac1")
(setf (get qb_device_srvs::SetPIDRequest :datatype-) "qb_device_srvs/SetPIDRequest")
(setf (get qb_device_srvs::SetPIDRequest :definition-)
      "# request
int32 id
int32 max_repeats
float32 p
float32 i
float32 d
---
# response
bool success
int32 failures
")

(setf (get qb_device_srvs::SetPIDResponse :md5sum-) "924f2bcd17c4f9d95c42467a96352ac1")
(setf (get qb_device_srvs::SetPIDResponse :datatype-) "qb_device_srvs/SetPIDResponse")
(setf (get qb_device_srvs::SetPIDResponse :definition-)
      "# request
int32 id
int32 max_repeats
float32 p
float32 i
float32 d
---
# response
bool success
int32 failures
")



(provide :qb_device_srvs/SetPID "924f2bcd17c4f9d95c42467a96352ac1")


