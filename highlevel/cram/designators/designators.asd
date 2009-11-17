; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

(asdf:defsystem cram/designators
  :name "cram/designators"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Implementation of designators."
  :long-description "Designators are the interface of cram do describe
                     all possible parameters that need reasoning. In particular this
                     includes object descriptions, locations and trajectories."
  :depends-on (:alexandria)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "desig")
             (:file "utils"))
            :serial t)))
