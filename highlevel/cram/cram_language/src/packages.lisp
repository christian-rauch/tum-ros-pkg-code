;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;;                     Nikolaus Demmel <demmeln@cs.tum.edu>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :cl-user)

(defpackage :cram-walker
  (:nicknames :walker)
  (:documentation 
   "A fairly basic code walker for use in CPL for expanding plans.")
  (:use #:common-lisp)
  (:export
   ;; plan tree
   #:plan-tree-node
   #:plan-tree-node-sexp
   #:plan-tree-node-parent
   #:plan-tree-node-children
   #:plan-tree-node-path
   #:find-plan-node
   ;; interface-functions
   #:expand-plan
   #:walk-with-tag-handler)
  (:shadow ;; Do this in case some lisp implementation defines
   ;; these cltl2 functions in the common-lisp package
   #:augment-environment
   #:parse-macro
   #:enclose)
  (:import-from #:alexandria
                #:rcurry))

(defpackage :cram-execution-trace
  (:nicknames :cet)
  (:documentation
   "The execution trace records plan execution to later analyse what happened
   during plan execution.")
  (:use :common-lisp
        :cl-store
        :portable-threads
        :alexandria
        :cram-utilities)
  (:export
   ;; durable-copy.lisp
   #:durable-copy
   ;; tracing.lisp
   #:traced-instance
   #:timestamp
   #:traced-id-instance
   #:id
   #:append-to-trace
   #:tracing-instance-mixin
   #:make-traced-instance
   #:trace-instance
   #:traced-value
   ;; episode-knwoledge.lisp
   #:throughout
   #:episode-knowledge
   #:live-episode-knowledge
   #:offline-episode-knowledge
   #:*episode-knowledge*
   #:get-top-level-episode-knowledge
   #:set-top-level-episode-knowledge
   #:episode-knowledge-zero-time
   #:episode-knowledge-max-time
   #:episode-knowledge-task-tree
   #:episode-knowledge-task-list
   #:episode-knowledge-goal-task-list
   #:episode-knowledge-fluent-trace-queue
   #:episode-knowledge-traced-fluent-names
   #:episode-knowledge-fluent-changes
   #:episode-knowledge-fluent-durations
   #:make-episode-knowledge
   #:reset-episode-knowledge
   #:save-episode-knowledge
   #:load-episode-knowledge
   #:with-episode-knowledge
   #:with-top-level-episode-knowledge
   #:with-offline-episode-knowledge
   ))

;;;; A few notes on the package setup.
;;;
;;; Cram, the language, is split up into two packages, CPL-IMPL and
;;; CPL. The former contains the language implementation whereas the
;;; latter is supposed to be :USEd.
;;;
;;; There are two notable differences:
;;;
;;;   i) CPL-IMPL only exports symbols which are specific to CRAM.
;;;
;;;      CPL additionally reexports all of CL. So users are supposed
;;;      to simply write (:USE :CPL), and to not include :CL there.
;;;
;;;  ii) The fluent operations are prefixed with "FL" in CPL-IMPL; 
;;;      however, in CPL, these are exported as +,-,*,/,etc.
;;; 
;;;      E.g. CPL:+ is actually CPL-IMPL:FL+.
;;;
;;; Caveat: 
;;;   CPL:EQL is not the same as CL:EQL. That means, if you want to
;;;   use, e.g., eql-specializers in a package which uses CPL, you
;;;   have to write CL:EQL explicitly.

#.(let ((cpl-symbols
         '(;; walker
           #:plan-tree-node #:plan-tree-node-sexp #:plan-tree-node-parent
           #:plan-tree-node-children #:plan-tree-node-path 
           #:find-plan-node
           #:expand-plan
           ;; fluent.lisp
           #:fluent
           #:value
           #:wait-for
           #:pulse
           #:whenever
           #:name
           ;; tracing-fluent.lisp
           #:make-fluent
           #:set-make-tracing-fluent
           #:traced-fluent
           #:traced-value
           ;; execution-trace
           #:throughout
           #:episode-knowledge
           #:live-episode-knowledge
           #:offline-episode-knowledge
           #:*episode-knowledge*
           #:get-top-level-episode-knowledge
           #:set-top-level-episode-knowledge
           #:episode-knowledge-zero-time
           #:episode-knowledge-max-time
           #:episode-knowledge-task-tree
           #:episode-knowledge-task-list
           #:episode-knowledge-goal-task-list
           #:episode-knowledge-fluent-trace-queue
           #:episode-knowledge-traced-fluent-names
           #:episode-knowledge-fluent-changes
           #:episode-knowledge-fluent-durations
           #:make-episode-knowledge
           #:reset-episode-knowledge
           #:save-episode-knowledge
           #:load-episode-knowledge
           #:with-episode-knowledge
           #:with-top-level-episode-knowledge
           #:with-offline-episode-knowledge
           ;; failures.lisp
           #:fail #:simple-plan-error #:rethrown-error
           #:plan-error #:plan-error-message #:plan-error-data
           #:with-failure-handling #:retry
           ;; task.lisp
           #:*current-task*
           #:status #:result
           #:suspend-protect #:without-suspension #:with-suspension #:on-suspension
           #:with-termination-handler #:ignore-termination #:without-termination
           ;; task-tree.lisp
           #:code
           #:code-parameters
           #:code-sexp
           #:code-function
           #:code-task
           #:task-tree-node
           #:task-tree-node-p
           #:task-tree-node-path
           #:task-tree-node-code
           #:task-tree-node-parent
           #:task-tree-node-children
           #:task-tree-node-effective-code
           #:with-task-tree-node
           #:make-task-tree-node
           #:replaceable-function
           #:make-task
           #:sub-task
           #:task
           #:clear-tasks
           #:*task-tree*
           #:flatten-task-tree
           #:task-tree-node-parameters
           #:task-tree-node-status-fluent
           #:task-tree-node-result
           #:goal-task-tree-node-p
           #:goal-task-tree-node-pattern
           #:goal-task-tree-node-parameter-bindings
           #:goal-task-tree-node-goal
           ;; base.lisp
           #:top-level #:seq #:par #:tag #:with-tags #:with-task-suspended
           #:pursue #:composite-failure #:try-all #:try-in-order #:tagged
           #:partial-order
           ;; plans.lisp
           #:def-top-level-plan #:get-top-level-task-tree #:def-plan
           ;; goals.lisp
           #:declare-goal #:def-goal #:goal #:register-goal))
        (fluent-ops
         '(;; fluent-net.lisp
           #:fl< #:fl> #:fl=  #:fl+ #:fl- #:fl* #:fl/
           #:fl-eq #:fl-eql #:fl-not #:fl-and #:fl-or 
           #:fl-pulsed #:fl-funcall))
        (cl-symbols
         (let (r) (do-external-symbols (s :cl r) (push s r)))))

    `(progn

       (defpackage :cram-language-implementation
         (:nicknames :cpl-impl)
         (:documentation "Internal implementation package of CPL.")
         (:use :common-lisp 
               :portable-threads
               :walker
               :cram-execution-trace
               :trivial-garbage
               :alexandria)
         (:export ,@cpl-symbols ,@fluent-ops))

       (defpackage :cram-language
         (:nicknames :cpl)
         (:documentation 
          "Main package of a new planning language similar to RPL, but
           implemented on the basis of macros and the portable-threads
           library")
         (:use :common-lisp
               :cram-language-implementation)
         (:export ,@cl-symbols 
                  ,@cpl-symbols
                  ;; Wrappers are defined in src/language.lisp.
                  #:< #:> #:+ #:- #:* #:/ #:= #:eq #:eql #:not
                  #:pulsed #:fl-and #:fl-or #:fl-funcall)
         (:shadow
          #:< #:> #:+ #:- #:* #:/ #:= #:eq #:eql #:not))))
