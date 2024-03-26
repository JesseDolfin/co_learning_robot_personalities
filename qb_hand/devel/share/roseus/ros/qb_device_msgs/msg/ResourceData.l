;; Auto-generated. Do not edit!


(when (boundp 'qb_device_msgs::ResourceData)
  (if (not (find-package "QB_DEVICE_MSGS"))
    (make-package "QB_DEVICE_MSGS"))
  (shadow 'ResourceData (find-package "QB_DEVICE_MSGS")))
(unless (find-package "QB_DEVICE_MSGS::RESOURCEDATA")
  (make-package "QB_DEVICE_MSGS::RESOURCEDATA"))

(in-package "ROS")
;;//! \htmlinclude ResourceData.msg.html


(defclass qb_device_msgs::ResourceData
  :super ros::object
  :slots (_name _position _velocity _effort _command ))

(defmethod qb_device_msgs::ResourceData
  (:init
   (&key
    ((:name __name) "")
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:effort __effort) 0.0)
    ((:command __command) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _effort (float __effort))
   (setq _command (float __command))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64 _position
    8
    ;; float64 _velocity
    8
    ;; float64 _effort
    8
    ;; float64 _command
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _effort
       (sys::poke _effort (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _command
       (sys::poke _command (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _effort
     (setq _effort (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _command
     (setq _command (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get qb_device_msgs::ResourceData :md5sum-) "378205f324e9e9cea98488f8746f3449")
(setf (get qb_device_msgs::ResourceData :datatype-) "qb_device_msgs/ResourceData")
(setf (get qb_device_msgs::ResourceData :definition-)
      "# Device-independent resource data message

string name
float64 position
float64 velocity
float64 effort
float64 command
")



(provide :qb_device_msgs/ResourceData "378205f324e9e9cea98488f8746f3449")


