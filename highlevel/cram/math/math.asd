; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

(asdf:defsystem cram/math
  :name "cram/math"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Some math utilities"
  :depends-on (alexandria gsll)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "random")
             (:file "functions")
             (:module "geometry"
                      :components
                      ((:file "vector")
                       (:file "orientation")
                       (:file "utils")
                       (:file "pose"))
                      :serial t))
            :serial t)))
