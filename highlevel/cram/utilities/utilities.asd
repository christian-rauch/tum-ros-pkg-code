; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

(asdf:defsystem cram/utilities
  :name "cram/utilities"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Pattern matching and other utilities."
  :long-description "Provides basic pattern matching, binding
                     handling, and other utilities used by other cram
                     components."
  :depends-on (alexandria
               portable-threads)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "utils")
             (:file "lazy")
             (:file "patmatch")
             (:file "data-pool")
             (:file "clos")
             (:file "ts-queue")
             (:file "time"))
            :serial t)))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram/utilities))))
  (asdf:operate 'asdf:load-op 'cram/utilities-tests)
  (asdf:operate 'asdf:test-op 'cram/utilities-tests))
