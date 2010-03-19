;;;
;;; Copyright (C) 2009 by Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package :kipla-reasoning)

;;; Note: #demmeln: not handling code replacements in any way atm.

(defun task-status-fluent-name (task-tree-node)
  (name (task-tree-node-status-fluent task-tree-node)))

(defgeneric extract-task-error (err)
  (:method ((err plan-error))
    err)
  (:method ((err rethrown-error))
    (rethrown-error err)))

(defun task-children (task)
  (mapcar #'cdr (task-tree-node-children task)))

;; Note: #demmeln: There is much potential for optimization, e.g. chaching the
;; task and fluent lists, only checking for correct type if task or fluents
;; are already bound etc...

(def-fact-group fluents
  ;; FLUENT
  (<- (fluent ?fluent)
    (not (bound ?fluent))
    (lisp-fun episode-knowledge-traced-fluent-names ?fluents)
    (member ?fluent ?fluents))

  (<- (fluent ?fluent)
    ;; bit of a hack to increase performance
    (bound ?fluent)
    (lisp-pred symbolp ?fluent))

  ;; HOLDS FLUENT-VALUE
  (<- (holds (fluent-value ?fluent ?value) ?t)
    (fluent ?fluent)
    (lisp-fun episode-knowledge-fluent-durations ?fluent ?durations)
    (member (?value . ?duration) ?durations)
    (duration-includes ?duration ?t)))

(def-fact-group tasks

  ;; TASK
  (<- (task ?task)
    (not (bound ?task))
    ;; Ignore all non goal tasks for high level reasoning
    (lisp-fun episode-knowledge-goal-task-list ?tasks)
    (member ?task ?tasks))

  (<- (task ?task)
    (bound ?task)
    (lisp-pred task-tree-node-p ?task))

  ;; SUBTASK
  (<- (subtask ?task ?subtask)
    (bound ?task)
    (lisp-fun task-children ?task ?children)
    (member ?subtask ?children))

  (<- (subtask ?task ?subtask)
    (not (bound ?task))
    (bound ?subtask)
    (lisp-fun task-tree-node-parent ?subtask ?task))

  (<- (subtask ?task ?subtask)
    (not (bound ?task))
    (not (bound ?subtask))
    (task ?task)
    (subtask ?task ?subtask))

  ;; SUBTASK+
  (<- (subtask+ ?task ?subtask)
    (subtask ?task ?subtask))

  (<- (subtask+ ?task ?subtask)
    (subtask ?task ?tmp)
    (subtask+ ?tmp ?subtask))

  ;; TASK-STATUS-FLUENT
  (<- (task-status-fluent ?task ?fluent)
    (task ?task)
    (lisp-fun task-status-fluent-name ?task ?fluent))

  ;; TASK-GOAL
  (<- (task-goal ?task ?goal)
    (task ?task)
    (lisp-fun goal-task-tree-node-goal ?task ?goal))

  ;; TASK-OUTCOME
  (<- (task-outcome ?task ?outcome)
    (member ?outcome (:succeeded :failed :evaporated))
    (holds (task-status ?task ?outcome) ?_))

  ;; TASK-RESULT
  (<- (task-result ?task ?result)
    (task ?task)
    (lisp-fun task-tree-node-result ?task ?result))

  ;; TASK-ERROR
  (<- (task-error ?task ?error)
    (task-outcome ?task :failed)
    (task-result ?task ?result)
    (lisp-fun extract-task-error ?result ?error))

  ;; ERROR-TYPE
  (<- (error-type ?error ?type)
    (bound ?error)
    (lisp-fun type-of ?error ?type))

  ;; HOLDS TASK-STATUS
  (<- (holds (task-status ?task ?status) ?t)
    (task-status-fluent ?task ?status-fluent)
    (holds (fluent-value ?status-fluent ?status) ?t)))
