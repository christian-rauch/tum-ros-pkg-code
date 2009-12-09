;;; Copyright 2009 Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions
;;; are met:
;;;
;;; 1. Redistributions of source code must retain the above copyright
;;;    notice, this list of conditions and the following disclaimer.
;;; 2. Redistributions in binary form must reproduce the above copyright
;;;    notice, this list of conditions and the following disclaimer in the
;;;    documentation and/or other materials provided with the distribution.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
;;; IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
;;; OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
;;; IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
;;; INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
;;; NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;;; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;;; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;;; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
;;; THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

(in-package :cl-user)

(asdf:operate 'asdf:load-op :lisp_asdf_manager/lisp_asdf_manager)

(asdf:defsystem cljlo/cljlo
    :name "cljlo/cljlo"
    :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
    :version "0.1"
    :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
    :license "BSD"
    :description "Common lisp interface to jlo service"

    :depends-on (roslisp
                 vision_srvs-srv
                 vision_msgs-msg
                 trivial-garbage
                 alexandria)

    :components
    ((:module "src"
              :components ((:file "package")
                           (:file "gc")
                           (:file "service")
                           (:file "jlo")
                           (:file "utils"))
              :serial t)))
