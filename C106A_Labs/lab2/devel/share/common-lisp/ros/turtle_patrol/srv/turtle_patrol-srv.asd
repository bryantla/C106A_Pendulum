
(cl:in-package :asdf)

(defsystem "turtle_patrol-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Patrol" :depends-on ("_package_Patrol"))
    (:file "_package_Patrol" :depends-on ("_package"))
  ))