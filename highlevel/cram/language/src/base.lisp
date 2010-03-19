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

(defmacro def-plan-macro (name lambda-list &body body)
  (setf (get name 'plan-type) :plan-macro)
  (setf (get name 'plan-lambda-list) lambda-list)
  (setf (get name 'plan-sexp) body)
  (let* ((documentation (when (stringp (car body))
                          (car body)))
         (body (if documentation
                   (cdr body)
                   body)))
    `(defmacro ,name ,lambda-list
       ,documentation
       `(cond ((not *current-task*)
               (error (format nil "'~a' form not inside a plan." ',',name)))
              (t
               ,,@body)))))

(defmacro with-parallel-childs ((running done failed) child-forms
                                &body watcher-body)
  "Execute child-forms in parallel and execute watcher-body whenever
   any child changes its status. The variables running, done and
   failed are lexically bound in watcher-body and are lists of all
   running, done and failed tasks. Please note that watcher-body can
   be terminated by a return call."
  `(with-task (:name "WITH-PARALLEL-CHILDS")
     ,@(loop with n = (length child-forms)
             for form in child-forms and i from 1
             collect `(make-instance 'task 
                        :name ',(format-gensym "[PARALLEL-CHILD-#~D/~D]-" i n)
                        :thread-fun (lambda () ,form)))
     (wait-for (fl-funcall #'every (compose #'not (curry #'eq :created))
                           (fl-funcall #'mapcar #'status (child-tasks *current-task*))))
     (block nil
       (whenever ((apply #'fl-pulsed (mapcar #'status (child-tasks *current-task*))))
         (multiple-value-bind (,running ,done ,failed)
             (loop
                for task in (child-tasks *current-task*)
                when (task-running-p task) collect task into running
                when (task-done-p task)    collect task into done
                when (task-failed-p task)  collect task into failed
                finally
                  (return (values running done failed)))
           ,@watcher-body)))))

(defmacro top-level (&body body)
  "Creates a new task, executes body in it and waits until it is
   finished. All plan macros can only be used within the dynamic
   scope of a top-level form."
  (with-gensyms (task)
    `(let ((*episode-knowledge* (or *episode-knowledge* (make-episode-knowledge))))
       (declare (special *episode-knowledge*))
       (reset-episode-knowledge)
       (let* ((*task-tree* (episode-knowledge-task-tree))
              (*current-task-tree-node* *task-tree*)
              (*current-path* (list)))
         (declare (special *task-tree* *current-task-tree-node* *current-path*))
         (when *current-task*
           (error "top-level calls cannot be nested."))
         (let ((,task (make-instance 'task
                        :name ',(gensym "[TOP-LEVEL]-")
                        :thread-fun (lambda () ,@body)
                        :ignore-no-parent t)))
           (with-failure-handling
               ((plan-error (e)
                  (error e))
                (error (e)
                  (error e)))
             (unwind-protect
                  (join-task ,task)
               (terminate ,task :evaporated))))))))

(def-plan-macro with-task ((&key (name "WITH-TASK")) &body body)
  "Executes body in a separate task and joins it."
  (with-gensyms (task)
    `(let ((,task (make-instance 'task
                    :name ',(gensym (format nil "[~a]-" name))
                    :thread-fun (lambda () ,@body))))
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
        `(let* ((,current-path *current-path*)
                ,@(mapcar (lambda (tag)
                            `(,tag (task ',tag (cons `(tagged ,',tag) ,current-path))))
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
                         tags))))))))

(def-plan-macro with-task-suspended (task &body body)
  "Execute body with 'task' being suspended."
  (with-gensyms (task-sym)
    `(let ((,task-sym ,task))
       (unwind-protect
            (progn
              (suspend ,task-sym)
              (wait-for (fl-eq (status ,task-sym) :suspended))
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
