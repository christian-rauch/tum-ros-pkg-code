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


(in-package :cpl)

(defmacro top-level (&body body)
  "Creates a new task, executes body in it and waits until it is
   finished. All plan macros can only be used within the dynamic
   scope of a top-level form."
  (with-gensyms (task)
    `(let* ((*task-tree* (or *task-tree* (make-task-tree-node)))
            (*current-task-tree-node* (or *current-task-tree-node* *task-tree*)))
       (declare (special *task-tree* *current-task-tree-node*))
       (when *current-task*
         (error "top-level calls cannot be nested."))
       (let ((,task (make-instance 'task
                      :thread-fun (lambda () ,@body)
                      :ignore-no-parent t)))
         (with-failure-handling
             ((plan-error (e)
                (error e))
              (error (e)
                (error e)))
             (unwind-protect
                  (join-task ,task)
               (terminate ,task :evaporated)))))))

(def-plan-macro with-task (&body body)
  "Executes body in a separate task and joins it."
  (with-gensyms (task)
    `(let ((,task (make-instance 'task
                    :thread-fun (lambda () ,@body))))
       (register-child *current-task* ,task)
       (join-task ,task))))

(defmacro seq (&body forms)
  "Executes forms sequentially. Fails if one fail. Succeeds if all
   succeed."
  `(progn ,@forms))

(def-plan-macro par (&body forms)
  "Executes forms in parallel. Fails if one fails. Succeeds if all
   succeed."
  `(with-parallel-childs (running-tasks finished-tasks failed-tasks)
       ,forms
     (cond (failed-tasks
            (fail (result (car failed-tasks))))
           ((not running-tasks)
            (return (result (car (last finished-tasks))))))))

(defmacro :tag (name &body body)
  "Create a tag named name in the current lexical scope."
  (declare (ignore body))
  (error (format nil ":tag '~a' used without a 'with-tags' environment." name)))

;;; - Don't nest with-tags calls, since code walking will mess up the result.
;;; - Don't use with-tags within macrole/symbol-macrolet/..., if those macros expand
;;;   to somethion containig (:tag ...) forms. They won't be picked up by the code
;;;   walker and thus not
;;;   handled properly.
;;; - Don't use with-tags withing flet/labels/let/... if those establish bindings that
;;;   shadow global macros that expand to something containing (:tag ...) forms, as
;;;   the code walker will assume the global macros and falsely pick up those tagged forms.
(def-plan-macro with-tags (&body body &environment lexenv)
  "Execute body with all tags bound to the corresponding lexically
   bound variables."
  (let ((tags (list))
        (tags-body `(progn ,@body)))
    (flet ((tags-handler (tag-name body)
             (declare (ignore body))
             (push tag-name tags)))
      (walk-with-tag-handler tags-body #'tags-handler lexenv)
      (with-gensyms (current-path)
        `(with-task
           (let* ((,current-path *current-path*)
                  ,@(mapcar (lambda (tag)
                              `(,tag (task (cons `(tagged ,',tag) ,current-path))))
                            tags))
             (declare (ignorable ,current-path))
             (macrolet ((:tag (name &body tag-body)
                          `(execute-task-tree-node
                            (register-task-code ',tag-body (lambda () ,@tag-body)
                                                :path (cons `(tagged ,',name) ,',current-path)))))
               (unwind-protect
                    ,tags-body
                 ,@(mapcar (lambda (tag)
                             `(terminate ,tag :evaporated))
                           tags)))))))))

(def-plan-macro with-task-blocked (task &body body)
  "Execute body with 'task' blocked."
  (with-gensyms (task-sym)
    `(let ((,task-sym ,task))
       (unwind-protect
            (progn
              (suspend ,task-sym)
              (wait-for (eq (status ,task-sym) :blocked))
              ,@body)
         (wake-up ,task-sym)))))

(def-plan-macro pursue (&body forms)
  "Execute forms in parallel. Succeed if one succeeds, fail if one
   fails."
  `(with-parallel-childs (running-tasks finished-tasks failed-tasks)
       ,forms
     (declare (ignore running-tasks))
     (cond (failed-tasks
            (fail (result (car failed-tasks))))
           (finished-tasks
            (return (result (car finished-tasks)))))))

(def-plan-macro try-all (&body forms)
  "Try forms in parallel. Succeed if one succeeds, fail if all fail.
   In the case of a failure, a condition of type 'composite-failure'
   is signaled, containing the list of all error messages and data."
  `(with-parallel-childs (running-tasks finished-tasks failed-tasks)
       ,forms
     (cond ((and (not running-tasks) (not finished-tasks) failed-tasks)
            (fail  (make-condition 'composite-failure
                                   :failures (mapcar #'result failed-tasks))))
           (finished-tasks
            (return (result (car (last finished-tasks))))))))

(def-plan-macro try-in-order (&body forms)
  "Execute forms sequentially. Succeed if one succeeds, fail if all fail.
   In case of failure, a composite-failure is signaled."
  (with-gensyms (failures)
    `(block nil
       (let ((,failures (list)))
         ,@(mapcar (lambda (form)
                     `(handler-case (return ,form)
                        (plan-error (err)
                          (push err ,failures))))
                   forms)
         (fail (make-condition 'composite-failure
                               :failures ,failures))))))

(def-plan-macro partial-order ((&body steps) &body orderings)
  (multiple-value-bind (bindings orders)
      (loop for (sym constraining-task constrained-task) in orderings
         for constraining-name = (gensym "CONSTRAINING-TASK-")
         for constrained-name = (gensym "CONSTRAINED-TASK-")
         unless (eq sym :order) do (error "Malformed ordering constraint.")
         nconc `((,constraining-name ,constraining-task)
                 (,constrained-name ,constrained-task)) into bindings
         collect (list constraining-name constrained-name) into orders
         finally (return (values bindings orders)))
    `(let ,bindings
       ,@(loop for (constraining constrained) in orders
            collect `(push (task-dead ,constraining)
                           (task-constraints ,constrained)))
       (par
         ,@steps))))
