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
  (:documentation "A fairly basic code walker for use in CPL for expanding plans.")
  (:nicknames :walker)
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

(defpackage :cram-language
  (:documentation "Main package of a new planning language similar to RPL, but implemented on the basis
                   of macros and the portable-threads library")
  (:nicknames :cpl)
  (:use #:common-lisp #:portable-threads #:walker :alexandria)
  (:export
   ;; walker
   #:plan-tree-node #:plan-tree-node-sexp #:plan-tree-node-parent
   #:plan-tree-node-children #:plan-tree-node-path #:find-plan-node
   #:expand-plan
   ;; fluent.lisp
   #:make-fluent #:fluent #:value #:wait-for #:pulse #:whenever
   #:name #:logged-fluent
   ;; fluent-net.lisp
   #:< #:> #:+ #:- #:* #:/ #:= #:eq #:eql #:not
   #:fl-and #:fl-or #:pulsed #:fl-funcall
   ;; failures.lisp
   #:fail #:plan-error #:simple-plan-error #:plan-error-message #:plan-error-data
   #:with-failure-handling #:retry
   ;; task.lisp
   #:status #:result
   #:suspend-protect #:on-suspend #:*current-task*
   #:with-termination-handler #:without-termination #:terminate-protect
   ;; task-tree.lisp
   #:code #:task-tree-node #:with-task-tree-node
   #:replaceable-function #:task-tree-node-effective-code #:make-task
   #:sub-task #:task #:clear-tasks #:task-tree-node #:replace-task-code
   #:flatten-task-tree #:task-tree-node-path #:*task-tree*
   #:task-tree-node-code #:code-parameters #:code-sexp #:code-function
   #:code-task #:flatten-task-tree
   ;; base.lisp
   #:top-level #:seq #:par #:tag #:with-tags #:with-task-blocked
   #:pursue #:composite-failure #:try-all #:try-in-order #:tagged
   #:partial-order
   ;; plans.lisp
   #:def-top-level-plan #:get-top-level-task-tree #:def-plan
   ;; goals.lisp
   #:declare-goal #:def-goal #:goal
   ;; time
   #:current-timestamp
   #:set-default-timestamp-function
   #:set-timestamp-function
   #:time-value-p
   ;; logging.lisp
   #:clear-instance-log #:get-logged-instances #:get-logged-timespan
   #:logged-instance #:make-logged-instance #:get-min-logged-time
   #:timestamp #:id #:logged-value)
  (:shadow
   ;; Symbols shadowed by fluent-nets
   #:< #:> #:+ #:- #:* #:/ #:= #:eq #:eql #:not))
