;; Auto-generated. Do not edit!


(when (boundp 'qb_device_srvs::SetCommands)
  (if (not (find-package "QB_DEVICE_SRVS"))
    (make-package "QB_DEVICE_SRVS"))
  (shadow 'SetCommands (find-package "QB_DEVICE_SRVS")))
(unless (find-package "QB_DEVICE_SRVS::SETCOMMANDS")
  (make-package "QB_DEVICE_SRVS::SETCOMMANDS"))
(unless (find-package "QB_DEVICE_SRVS::SETCOMMANDSREQUEST")
  (make-package "QB_DEVICE_SRVS::SETCOMMANDSREQUEST"))
(unless (find-package "QB_DEVICE_SRVS::SETCOMMANDSRESPONSE")
  (make-package "QB_DEVICE_SRVS::SETCOMMANDSRESPONSE"))

(in-package "ROS")





(defclass qb_device_srvs::SetCommandsRequest
  :super ros::object
  :slots (_id _max_repeats _set_commands _set_commands_async _commands ))

(defmethod qb_device_srvs::SetCommandsRequest
  (:init
   (&key
    ((:id __id) 0)
    ((:max_repeats __max_repeats) 0)
    ((:set_commands __set_commands) nil)
    ((:set_commands_async __set_commands_async) nil)
    ((:commands __commands) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _max_repeats (round __max_repeats))
   (setq _set_commands __set_commands)
   (setq _set_commands_async __set_commands_async)
   (setq _commands __commands)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:max_repeats
   (&optional __max_repeats)
   (if __max_repeats (setq _max_repeats __max_repeats)) _max_repeats)
  (:set_commands
   (&optional (__set_commands :null))
   (if (not (eq __set_commands :null)) (setq _set_commands __set_commands)) _set_commands)
  (:set_commands_async
   (&optional (__set_commands_async :null))
   (if (not (eq __set_commands_async :null)) (setq _set_commands_async __set_commands_async)) _set_commands_async)
  (:commands
   (&optional __commands)
   (if __commands (setq _commands __commands)) _commands)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _max_repeats
    4
    ;; bool _set_commands
    1
    ;; bool _set_commands_async
    1
    ;; int16[] _commands
    (* 2    (length _commands)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _max_repeats
       (write-long _max_repeats s)
     ;; bool _set_commands
       (if _set_commands (write-byte -1 s) (write-byte 0 s))
     ;; bool _set_commands_async
       (if _set_commands_async (write-byte -1 s) (write-byte 0 s))
     ;; int16[] _commands
     (write-long (length _commands) s)
     (dotimes (i (length _commands))
       (write-word (elt _commands i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_repeats
     (setq _max_repeats (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _set_commands
     (setq _set_commands (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _set_commands_async
     (setq _set_commands_async (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16[] _commands
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _commands (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _commands i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(defclass qb_device_srvs::SetCommandsResponse
  :super ros::object
  :slots (_success _failures ))

(defmethod qb_device_srvs::SetCommandsResponse
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

(defclass qb_device_srvs::SetCommands
  :super ros::object
  :slots ())

(setf (get qb_device_srvs::SetCommands :md5sum-) "2a24c554c16e33a4da324c504c12f0f4")
(setf (get qb_device_srvs::SetCommands :datatype-) "qb_device_srvs/SetCommands")
(setf (get qb_device_srvs::SetCommands :request) qb_device_srvs::SetCommandsRequest)
(setf (get qb_device_srvs::SetCommands :response) qb_device_srvs::SetCommandsResponse)

(defmethod qb_device_srvs::SetCommandsRequest
  (:response () (instance qb_device_srvs::SetCommandsResponse :init)))

(setf (get qb_device_srvs::SetCommandsRequest :md5sum-) "2a24c554c16e33a4da324c504c12f0f4")
(setf (get qb_device_srvs::SetCommandsRequest :datatype-) "qb_device_srvs/SetCommandsRequest")
(setf (get qb_device_srvs::SetCommandsRequest :definition-)
      "# request
int32 id
int32 max_repeats
bool set_commands
bool set_commands_async
int16[] commands
---
# response
bool success
int32 failures
")

(setf (get qb_device_srvs::SetCommandsResponse :md5sum-) "2a24c554c16e33a4da324c504c12f0f4")
(setf (get qb_device_srvs::SetCommandsResponse :datatype-) "qb_device_srvs/SetCommandsResponse")
(setf (get qb_device_srvs::SetCommandsResponse :definition-)
      "# request
int32 id
int32 max_repeats
bool set_commands
bool set_commands_async
int16[] commands
---
# response
bool success
int32 failures
")



(provide :qb_device_srvs/SetCommands "2a24c554c16e33a4da324c504c12f0f4")


