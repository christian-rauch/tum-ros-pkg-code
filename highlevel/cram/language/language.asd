; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

(asdf:defsystem cram/language
  :name "cram-language"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Coginitive plan language"
  :long-description "cram-language is a new plan language, based on macros
                     and the portable-threads library."

  :depends-on (portable-threads
               trivial-garbage
               alexandria
               cram/utilities)

  :components
  ((:module "src"
    :components
    ((:file "packages")
     (:file "time")
     (:file "ts-queue")
     (:file "object-identities")
     (:file "persistent-copy")
     (:file "logging")
     (:module "walker"
              :components (#+sbcl (:file "augment-environment-sbcl-patch")
                                  ;; TODO: Remove this, when SBCL 1.0.31 hopefully
                                  ;;       includes sb-cltl2::augment-environment
                                  (:file "env")
                                  (:file "env-impl-specific")
                                  (:file "plan-tree")
                                  (:file "walker")
                                  (:file "interface"))
              :serial t)
     (:file "task")
     (:file "failures")
     (:file "fluent")
     (:file "fluent-net")
     (:file "task-tree")
     (:file "task.implementation")
     (:file "utils")
     (:file "base")
     (:file "plans")
     (:file "goals"))
    :serial t)))
