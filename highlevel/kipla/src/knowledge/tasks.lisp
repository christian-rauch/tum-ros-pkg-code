;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Nikolaus Demmel <demmeln@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :kipla-reasoning)

;;; #demmeln: not handling code replacements in any way atm.
;;; #demmeln: some of these defuns should be put in cram/language.

(defun goal-task-p (task-tree-node)
  (when (typep task-tree-node 'task-tree-node)
    (let ((p (task-tree-node-path task-tree-node)))
      ;; assume cpl::goal has been importet in the current package
      (eq 'goal
          (and (consp p)
               (consp (car p))
               (caar p))))))

;;; Ignore all non goal tasks for high level reasoning
(defun get-all-goal-tasks ()
  (remove-if-not #'goal-task-p (flatten-task-tree *task-tree*)))

(defun task-parameters (task-tree-node)
  (code-parameters (task-tree-node-code task-tree-node)))

(defun goal-task-pattern (task-tree-node)
  (when (goal-task-p task-tree-node)
    (cadar (task-tree-node-path task-tree-node))))

(defun goal-task-parameter-bindings (task-tree-node)
  (when (goal-task-p task-tree-node)
    (let ((params (task-parameters task-tree-node))
          (pattern (goal-task-pattern task-tree-node)))
      (mapcar (lambda (var value) (cons var value))
              (vars-in pattern)
              params))))

(defun extract-task-goal (task-tree-node)
  (when (goal-task-p task-tree-node)
    (substitute-vars (goal-task-pattern task-tree-node)
                     (goal-task-parameter-bindings task-tree-node))))

(defun task-status-fluent (task-tree-node)
  (let ((code (task-tree-node-code task-tree-node)))
    (when code
      (status (code-task code)))))

(defun task-status-changes (task-tree-node)
  (let ((status-fluent (task-status-fluent task-tree-node)))
    (when status-fluent
      (let* ((fluent-name (name status-fluent))
             (logged-fluents (remove-if-not (lambda (x)
                                              (and (typep x 'logged-fluent)
                                                   (eq (name x) fluent-name)))
                                            (get-logged-instances))))
        (setf logged-fluents (sort logged-fluents #'< :key #'timestamp))
        (mapcar (lambda (x)
                  (cons (logged-value x)
                        (timestamp x)))
                logged-fluents)))))

(defun status-changes->status-durations (changes)
  (multiple-value-bind (min-time max-time)
      (get-logged-timespan)
    (declare (ignore min-time))
    (maplist (lambda (x)
               (cons (caar x)
                     `(throughout ,(cdar x)
                                  ,(if (cdr x)
                                       (cdadr x)
                                       max-time))))
             changes)))

(defun task-status-durations (task-tree-node)
  (status-changes->status-durations (task-status-changes task-tree-node)))

(def-fact-group tasks

  (<- (task ?task)
    (lisp-fun get-all-goal-tasks ?tasks)
    (member ?task ?tasks))

  (<- (task-goal ?task ?goal)
    (task ?task)
    (lisp-fun extract-task-goal ?task ?goal))

  (<- (holds (task-status ?task ?status) ?t)
    (task ?task)
    (lisp-fun task-status-durations ?task ?status-durations)
    (member (?status . ?duration) ?status-durations)
    (duration-includes ?duration ?t)))