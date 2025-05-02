
(cl:in-package :asdf)

(defsystem "human_detection_pipeline-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "HumanTrackingData" :depends-on ("_package_HumanTrackingData"))
    (:file "_package_HumanTrackingData" :depends-on ("_package"))
    (:file "HumansList" :depends-on ("_package_HumansList"))
    (:file "_package_HumansList" :depends-on ("_package"))
  ))