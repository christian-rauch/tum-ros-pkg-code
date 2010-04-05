;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cram-language
  :name "cram-language"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Coginitive plan language"
  :long-description "cram-language is a new plan language, based on macros
                     and the portable-threads library."

  :depends-on (portable-threads
               cl-store
               trivial-garbage
               alexandria
               cram-utilities)

  :components
  ((:module "src"
    :components
    ((:file "packages")
     (:file "utils")
     (:module "walker"
              :components
              ((:file "augment-environment-sbcl-patch")
               (:file "env")
               (:file "env-impl-specific")
               (:file "plan-tree")
               (:file "walker")
               (:file "interface"))
              :serial t)
     (:file "task")
     (:file "failures")
     (:file "task-tree")
     (:file "fluent")
     (:file "fluent-net")
     (:file "task.implementation")
     (:module "execution-trace"
              :components
              ((:file "object-identities")
               (:file "durable-copy")
               (:file "episode-knowledge")
               (:file "offline-task")
               (:file "episode-knowledge-backend")
               (:file "tracing"))
              :serial t)
     (:file "tracing-fluent")
     (:file "base")
     (:file "plans")
     (:file "goals")
     (:file "language")
     (:file "swank-indentation"))
    :serial t)))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-language))))
  (asdf:operate 'asdf:load-op 'cram-language-tests)
  (asdf:operate 'asdf:test-op 'cram-language-tests))
