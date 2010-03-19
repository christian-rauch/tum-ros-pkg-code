;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:operate 'asdf:load-op "lisp_asdf_manager/lisp_asdf_manager")

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
                         (:file "suite")
                         (:file "walker")
                         (:file "fluents")
                         (:file "language-base")
                         (:file "execution-trace"))
            :serial t)))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram/language-tests))))
  (flet ((symbol (pkg name)
           (intern (string name) (find-package pkg))))
    (funcall (symbol :5am :run!) (symbol :cpl-tests :language))))
