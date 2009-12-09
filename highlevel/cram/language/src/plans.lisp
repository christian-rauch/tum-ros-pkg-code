;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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


(in-package :cpl-impl)

;;; We need to manage the task trees somehow. The idea is to have a
;;; hash-table containing one task-tree for every top-level
;;; plan. Otherwise, the task-tree management would become really
;;; really messy.

(defvar *top-level-plan-task-trees* (make-hash-table :test 'cl:eq))

;;; Don't have (certain kinds of) surrounding flet/lables/macrolet/symbol-macrolet/let/...
;;; when using def-top-level-plan or def-plan. They could mess with (with-tags ...) or
;;; shadow globally defined plans, which would not be picked up by with-tags/expand-plan.
;;; See the comment before the definition of with-tags for more details.

(defmacro def-top-level-plan (name args &body body)
  "Defines a top-level plan. Every top-level plan has its own
   task-tree."
  (setf (gethash name *top-level-plan-task-trees*)
        (make-task-tree-node))
  (setf (get name 'plan-type) :top-level-plan)
  (setf (get name 'plan-lambda-list) args)
  (setf (get name 'plan-sexp) body)
  (with-gensyms (call-args)
    `(defun ,name (&rest ,call-args)
       (let* ((*task-tree* (gethash ',name *top-level-plan-task-trees*))
              (*current-task-tree-node* *task-tree*)
              (*current-path* (list)))
         (declare (special *task-tree* *current-task-tree-node*
                           *current-path*))
         (clear-tasks *task-tree*)
         (with-task
           (replaceable-function ,name ,args ,call-args `(top-level ,',name)
             (with-tags
               ,@body)))))))

(defun get-top-level-task-tree (name)
  "Returns the task-tree of the top-level plan."
  (gethash name *top-level-plan-task-trees*))

(defmacro def-plan (name lambda-list &rest body)
  "Defines a plan. All functions that should appear in the task-tree
   must be defined with def-plan."
  (setf (get name 'plan-type) :plan)
  (setf (get name 'plan-lambda-list) lambda-list)
  (setf (get name 'plan-sexp) body)
  (with-gensyms (call-args)
    `(defun ,name (&rest ,call-args)
       (with-task
         (replaceable-function ,name ,lambda-list ,call-args (list ',name)
           (with-tags
             ,@body))))))
