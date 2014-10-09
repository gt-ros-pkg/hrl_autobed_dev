
(cl:in-package :asdf)

(defsystem "hrl_autobed_dev-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "add_bed_config" :depends-on ("_package_add_bed_config"))
    (:file "_package_add_bed_config" :depends-on ("_package"))
  ))