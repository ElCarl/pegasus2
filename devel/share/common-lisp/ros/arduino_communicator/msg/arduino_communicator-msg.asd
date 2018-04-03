
(cl:in-package :asdf)

(defsystem "arduino_communicator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "EncoderCounts" :depends-on ("_package_EncoderCounts"))
    (:file "_package_EncoderCounts" :depends-on ("_package"))
  ))