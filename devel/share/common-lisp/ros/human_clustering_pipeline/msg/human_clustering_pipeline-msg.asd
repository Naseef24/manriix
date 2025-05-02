
(cl:in-package :asdf)

(defsystem "human_clustering_pipeline-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "OptimalPosition" :depends-on ("_package_OptimalPosition"))
    (:file "_package_OptimalPosition" :depends-on ("_package"))
  ))