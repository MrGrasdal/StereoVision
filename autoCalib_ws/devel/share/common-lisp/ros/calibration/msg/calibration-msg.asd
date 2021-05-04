
(cl:in-package :asdf)

(defsystem "calibration-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "gnssGGA" :depends-on ("_package_gnssGGA"))
    (:file "_package_gnssGGA" :depends-on ("_package"))
    (:file "gnssGGA_status" :depends-on ("_package_gnssGGA_status"))
    (:file "_package_gnssGGA_status" :depends-on ("_package"))
    (:file "orientation" :depends-on ("_package_orientation"))
    (:file "_package_orientation" :depends-on ("_package"))
  ))