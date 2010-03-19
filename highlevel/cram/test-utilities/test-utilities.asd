; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

(asdf:defsystem cram/test-utilities
  :name "cram/test-utilities"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Utilities for writing unit tests."
  :depends-on (alexandria)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "utils"))
            :serial t)))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram/test-utilities))))
  (asdf:operate 'asdf:load-op 'cram/test-utilities-tests)
  (asdf:operate 'asdf:test-op 'cram/test-utilities-tests))
