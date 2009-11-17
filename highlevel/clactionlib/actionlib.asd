;;; Copyright 2009 Piotr Esden-Tempski <esdentem@cs.tum.edu>
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

(asdf:operate 'load-op :lisp_asdf_manager/lisp_asdf_manager)

(asdf:defsystem clactionlib/actionlib
    :name "clactionlib/actionlib"
    :author "Piotr Esden-Tempski <esdentem@cs.tum.edu>"
    :version "0.1"
    :maintainer "Piotr Esden-Tempski <esdentem@cs.tum.edu>"
    :license "BSD"
    :description "Common lisp implementation of the action interface"
    :long-description "Common lisp implementation of the action interface"

    :depends-on (roslisp
                 roslib-msg
                 std_msgs-msg
                 actionlib_msgs-msg
                 portable-threads
                 trivial-garbage)

    :components
    ((:module "src"
              :components ((:file "packages")
                           (:file "utils")
                           (:file "state-machine")
                           (:file "goal-handle")
                           (:file "goal-manager")
                           (:file "action-client"))
              :serial t)))
