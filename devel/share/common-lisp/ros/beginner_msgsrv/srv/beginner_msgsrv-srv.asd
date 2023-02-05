
(cl:in-package :asdf)

(defsystem "beginner_msgsrv-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddTwoInt" :depends-on ("_package_AddTwoInt"))
    (:file "_package_AddTwoInt" :depends-on ("_package"))
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
  ))