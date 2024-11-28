
(cl:in-package :asdf)

(defsystem "mi_robot_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "HandData" :depends-on ("_package_HandData"))
    (:file "_package_HandData" :depends-on ("_package"))
  ))