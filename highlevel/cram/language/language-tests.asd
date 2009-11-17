; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

(asdf:defsystem cram/language-tests
  :name "cram-language-tests"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Test-suite for cram-language"

  :depends-on (cram/language fiveam alexandria)
  :components
  ((:module "tests"
            :components ((:file "package")
                         (:file "test-utils")
                         (:file "walker")
                         (:file "fluents")
                         (:file "language-base"))
            :serial t)))
