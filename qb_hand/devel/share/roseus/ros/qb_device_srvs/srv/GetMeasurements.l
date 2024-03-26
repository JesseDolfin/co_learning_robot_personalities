;; Auto-generated. Do not edit!


(when (boundp 'qb_device_srvs::GetMeasurements)
  (if (not (find-package "QB_DEVICE_SRVS"))
    (make-package "QB_DEVICE_SRVS"))
  (shadow 'GetMeasurements (find-package "QB_DEVICE_SRVS")))
(unless (find-package "QB_DEVICE_SRVS::GETMEASUREMENTS")
  (make-package "QB_DEVICE_SRVS::GETMEASUREMENTS"))
(unless (find-package "QB_DEVICE_SRVS::GETMEASUREMENTSREQUEST")
  (make-package "QB_DEVICE_SRVS::GETMEASUREMENTSREQUEST"))
(unless (find-package "QB_DEVICE_SRVS::GETMEASUREMENTSRESPONSE")
  (make-package "QB_DEVICE_SRVS::GETMEASUREMENTSRESPONSE"))

(in-package "ROS")





(defclass qb_device_srvs::GetMeasurementsRequest
  :super ros::object
  :slots (_id _max_repeats _get_positions _get_currents _get_distinct_packages _get_commands ))

(defmethod qb_device_srvs::GetMeasurementsRequest
  (:init
   (&key
    ((:id __id) 0)
    ((:max_repeats __max_repeats) 0)
    ((:get_positions __get_positions) nil)
    ((:get_currents __get_currents) nil)
    ((:get_distinct_packages __get_distinct_packages) nil)
    ((:get_commands __get_commands) nil)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _max_repeats (round __max_repeats))
   (setq _get_positions __get_positions)
   (setq _get_currents __get_currents)
   (setq _get_distinct_packages __get_distinct_packages)
   (setq _get_commands __get_commands)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:max_repeats
   (&optional __max_repeats)
   (if __max_repeats (setq _max_repeats __max_repeats)) _max_repeats)
  (:get_positions
   (&optional (__get_positions :null))
   (if (not (eq __get_positions :null)) (setq _get_positions __get_positions)) _get_positions)
  (:get_currents
   (&optional (__get_currents :null))
   (if (not (eq __get_currents :null)) (setq _get_currents __get_currents)) _get_currents)
  (:get_distinct_packages
   (&optional (__get_distinct_packages :null))
   (if (not (eq __get_distinct_packages :null)) (setq _get_distinct_packages __get_distinct_packages)) _get_distinct_packages)
  (:get_commands
   (&optional (__get_commands :null))
   (if (not (eq __get_commands :null)) (setq _get_commands __get_commands)) _get_commands)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _max_repeats
    4
    ;; bool _get_positions
    1
    ;; bool _get_currents
    1
    ;; bool _get_distinct_packages
    1
    ;; bool _get_commands
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _max_repeats
       (write-long _max_repeats s)
     ;; bool _get_positions
       (if _get_positions (write-byte -1 s) (write-byte 0 s))
     ;; bool _get_currents
       (if _get_currents (write-byte -1 s) (write-byte 0 s))
     ;; bool _get_distinct_packages
       (if _get_distinct_packages (write-byte -1 s) (write-byte 0 s))
     ;; bool _get_commands
       (if _get_commands (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_repeats
     (setq _max_repeats (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _get_positions
     (setq _get_positions (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _get_currents
     (setq _get_currents (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _get_distinct_packages
     (setq _get_distinct_packages (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _get_commands
     (setq _get_commands (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass qb_device_srvs::GetMeasurementsResponse
  :super ros::object
  :slots (_success _failures _positions _currents _commands _stamp ))

(defmethod qb_device_srvs::GetMeasurementsResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:failures __failures) 0)
    ((:positions __positions) (make-array 0 :initial-element 0 :element-type :integer))
    ((:currents __currents) (make-array 0 :initial-element 0 :element-type :integer))
    ((:commands __commands) (make-array 0 :initial-element 0 :element-type :integer))
    ((:stamp __stamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _failures (round __failures))
   (setq _positions __positions)
   (setq _currents __currents)
   (setq _commands __commands)
   (setq _stamp __stamp)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:failures
   (&optional __failures)
   (if __failures (setq _failures __failures)) _failures)
  (:positions
   (&optional __positions)
   (if __positions (setq _positions __positions)) _positions)
  (:currents
   (&optional __currents)
   (if __currents (setq _currents __currents)) _currents)
  (:commands
   (&optional __commands)
   (if __commands (setq _commands __commands)) _commands)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _failures
    4
    ;; int16[] _positions
    (* 2    (length _positions)) 4
    ;; int16[] _currents
    (* 2    (length _currents)) 4
    ;; int16[] _commands
    (* 2    (length _commands)) 4
    ;; time _stamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _failures
       (write-long _failures s)
     ;; int16[] _positions
     (write-long (length _positions) s)
     (dotimes (i (length _positions))
       (write-word (elt _positions i) s)
       )
     ;; int16[] _currents
     (write-long (length _currents) s)
     (dotimes (i (length _currents))
       (write-word (elt _currents i) s)
       )
     ;; int16[] _commands
     (write-long (length _commands) s)
     (dotimes (i (length _commands))
       (write-word (elt _commands i) s)
       )
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _failures
     (setq _failures (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _positions i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16[] _currents
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _currents (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _currents i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16[] _commands
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _commands (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _commands i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass qb_device_srvs::GetMeasurements
  :super ros::object
  :slots ())

(setf (get qb_device_srvs::GetMeasurements :md5sum-) "61d005acb1e04c16b9b33b19436d5ede")
(setf (get qb_device_srvs::GetMeasurements :datatype-) "qb_device_srvs/GetMeasurements")
(setf (get qb_device_srvs::GetMeasurements :request) qb_device_srvs::GetMeasurementsRequest)
(setf (get qb_device_srvs::GetMeasurements :response) qb_device_srvs::GetMeasurementsResponse)

(defmethod qb_device_srvs::GetMeasurementsRequest
  (:response () (instance qb_device_srvs::GetMeasurementsResponse :init)))

(setf (get qb_device_srvs::GetMeasurementsRequest :md5sum-) "61d005acb1e04c16b9b33b19436d5ede")
(setf (get qb_device_srvs::GetMeasurementsRequest :datatype-) "qb_device_srvs/GetMeasurementsRequest")
(setf (get qb_device_srvs::GetMeasurementsRequest :definition-)
      "# request
int32 id
int32 max_repeats
bool get_positions
bool get_currents
bool get_distinct_packages
bool get_commands
---
# response
bool success
int32 failures
int16[] positions
int16[] currents
int16[] commands
time stamp
")

(setf (get qb_device_srvs::GetMeasurementsResponse :md5sum-) "61d005acb1e04c16b9b33b19436d5ede")
(setf (get qb_device_srvs::GetMeasurementsResponse :datatype-) "qb_device_srvs/GetMeasurementsResponse")
(setf (get qb_device_srvs::GetMeasurementsResponse :definition-)
      "# request
int32 id
int32 max_repeats
bool get_positions
bool get_currents
bool get_distinct_packages
bool get_commands
---
# response
bool success
int32 failures
int16[] positions
int16[] currents
int16[] commands
time stamp
")



(provide :qb_device_srvs/GetMeasurements "61d005acb1e04c16b9b33b19436d5ede")


