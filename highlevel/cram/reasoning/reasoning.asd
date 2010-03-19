; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

(asdf:defsystem cram/reasoning
  :name "cram/reasoning"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Prolog-like language interpreter and reasoning framework."
  :long-description "Prolog-like language interpreter for keeping
                     dynamic knowledge bases and integrating other
                     sources of knowledge such as PROLOG through a
                     foreign language interface. It also provides an
                     implementation of the RETE algorithm."
  :depends-on (alexandria
               cram/utilities)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "fact-groups")
             (:file "prolog-facts")
             (:file "prolog-handlers")
             (:file "prolog")
             (:module "rete"
                      :components
                      ((:file "rete")
                       (:file "alpha-node")
                       (:file "alpha-memory-node")
                       (:file "alpha-network")
                       (:file "token")
                       (:file "beta-join-node")
                       (:file "prolog-node")
                       (:file "productions"))
                      :serial t)
             (:file "swank-indentation"))
            :serial t)))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram/reasoning))))
  (asdf:operate 'asdf:load-op 'cram/reasoning-tests)
  (asdf:operate 'asdf:test-op 'cram/reasoning-tests))
