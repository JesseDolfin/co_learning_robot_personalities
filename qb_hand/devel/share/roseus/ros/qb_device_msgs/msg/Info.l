;; Auto-generated. Do not edit!


(when (boundp 'qb_device_msgs::Info)
  (if (not (find-package "QB_DEVICE_MSGS"))
    (make-package "QB_DEVICE_MSGS"))
  (shadow 'Info (find-package "QB_DEVICE_MSGS")))
(unless (find-package "QB_DEVICE_MSGS::INFO")
  (make-package "QB_DEVICE_MSGS::INFO"))

(in-package "ROS")
;;//! \htmlinclude Info.msg.html


(defclass qb_device_msgs::Info
  :super ros::object
  :slots (_id _serial_port _max_repeats _get_positions _get_currents _get_distinct_packages _set_commands _set_commands_async _position_limits _encoder_resolutions ))

(defmethod qb_device_msgs::Info
  (:init
   (&key
    ((:id __id) 0)
    ((:serial_port __serial_port) "")
    ((:max_repeats __max_repeats) 0)
    ((:get_positions __get_positions) nil)
    ((:get_currents __get_currents) nil)
    ((:get_distinct_packages __get_distinct_packages) nil)
    ((:set_commands __set_commands) nil)
    ((:set_commands_async __set_commands_async) nil)
    ((:position_limits __position_limits) (make-array 0 :initial-element 0 :element-type :integer))
    ((:encoder_resolutions __encoder_resolutions) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _serial_port (string __serial_port))
   (setq _max_repeats (round __max_repeats))
   (setq _get_positions __get_positions)
   (setq _get_currents __get_currents)
   (setq _get_distinct_packages __get_distinct_packages)
   (setq _set_commands __set_commands)
   (setq _set_commands_async __set_commands_async)
   (setq _position_limits __position_limits)
   (setq _encoder_resolutions __encoder_resolutions)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serial_port
   (&optional __serial_port)
   (if __serial_port (setq _serial_port __serial_port)) _serial_port)
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
  (:set_commands
   (&optional (__set_commands :null))
   (if (not (eq __set_commands :null)) (setq _set_commands __set_commands)) _set_commands)
  (:set_commands_async
   (&optional (__set_commands_async :null))
   (if (not (eq __set_commands_async :null)) (setq _set_commands_async __set_commands_async)) _set_commands_async)
  (:position_limits
   (&optional __position_limits)
   (if __position_limits (setq _position_limits __position_limits)) _position_limits)
  (:encoder_resolutions
   (&optional __encoder_resolutions)
   (if __encoder_resolutions (setq _encoder_resolutions __encoder_resolutions)) _encoder_resolutions)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; string _serial_port
    4 (length _serial_port)
    ;; int32 _max_repeats
    4
    ;; bool _get_positions
    1
    ;; bool _get_currents
    1
    ;; bool _get_distinct_packages
    1
    ;; bool _set_commands
    1
    ;; bool _set_commands_async
    1
    ;; int32[] _position_limits
    (* 4    (length _position_limits)) 4
    ;; uint8[] _encoder_resolutions
    (* 1    (length _encoder_resolutions)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; string _serial_port
       (write-long (length _serial_port) s) (princ _serial_port s)
     ;; int32 _max_repeats
       (write-long _max_repeats s)
     ;; bool _get_positions
       (if _get_positions (write-byte -1 s) (write-byte 0 s))
     ;; bool _get_currents
       (if _get_currents (write-byte -1 s) (write-byte 0 s))
     ;; bool _get_distinct_packages
       (if _get_distinct_packages (write-byte -1 s) (write-byte 0 s))
     ;; bool _set_commands
       (if _set_commands (write-byte -1 s) (write-byte 0 s))
     ;; bool _set_commands_async
       (if _set_commands_async (write-byte -1 s) (write-byte 0 s))
     ;; int32[] _position_limits
     (write-long (length _position_limits) s)
     (dotimes (i (length _position_limits))
       (write-long (elt _position_limits i) s)
       )
     ;; uint8[] _encoder_resolutions
     (write-long (length _encoder_resolutions) s)
     (princ _encoder_resolutions s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _serial_port
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _serial_port (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _max_repeats
     (setq _max_repeats (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _get_positions
     (setq _get_positions (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _get_currents
     (setq _get_currents (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _get_distinct_packages
     (setq _get_distinct_packages (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _set_commands
     (setq _set_commands (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _set_commands_async
     (setq _set_commands_async (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32[] _position_limits
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position_limits (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _position_limits i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint8[] _encoder_resolutions
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _encoder_resolutions (make-array n :element-type :char))
     (replace _encoder_resolutions buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get qb_device_msgs::Info :md5sum-) "ee93ec9c1a360ac561916c3deecf8486")
(setf (get qb_device_msgs::Info :datatype-) "qb_device_msgs/Info")
(setf (get qb_device_msgs::Info :definition-)
      "# Standard device-independent info message

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
")



(provide :qb_device_msgs/Info "ee93ec9c1a360ac561916c3deecf8486")


