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
  :depends-on (alexandria)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "utils")
             (:file "lazy")
             (:file "patmatch"))
            :serial t)))