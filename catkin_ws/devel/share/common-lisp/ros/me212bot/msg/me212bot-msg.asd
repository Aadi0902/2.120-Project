
(cl:in-package :asdf)

(defsystem "me212bot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WheelCmdVel" :depends-on ("_package_WheelCmdVel"))
    (:file "_package_WheelCmdVel" :depends-on ("_package"))
  ))