;;;; -*- Mode: LISP -*-

(in-package :asdf)

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

(defsystem "cram/liswip"
  :author "Lars Kunze <kunzel@in.tum.de>"
  :version "0.1"
  :maintainer "Lars Kunze <kunzel@in.tum.de>"
  :license "BSD"
  :description "CFFI-Based Common Lisp <-> SWI Prolog interface."

  :depends-on (:cffi)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "liswip"))
            :serial t)))
