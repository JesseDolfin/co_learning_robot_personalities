
(cl:in-package :asdf)

(defsystem "qb_device_srvs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :qb_device_msgs-msg
)
  :components ((:file "_package")
    (:file "GetMeasurements" :depends-on ("_package_GetMeasurements"))
    (:file "_package_GetMeasurements" :depends-on ("_package"))
    (:file "InitializeDevice" :depends-on ("_package_InitializeDevice"))
    (:file "_package_InitializeDevice" :depends-on ("_package"))
    (:file "SetCommands" :depends-on ("_package_SetCommands"))
    (:file "_package_SetCommands" :depends-on ("_package"))
    (:file "SetControlMode" :depends-on ("_package_SetControlMode"))
    (:file "_package_SetControlMode" :depends-on ("_package"))
    (:file "SetPID" :depends-on ("_package_SetPID"))
    (:file "_package_SetPID" :depends-on ("_package"))
    (:file "Trigger" :depends-on ("_package_Trigger"))
    (:file "_package_Trigger" :depends-on ("_package"))
  ))