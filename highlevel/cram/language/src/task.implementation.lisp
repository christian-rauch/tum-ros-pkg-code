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

(defclass task ()
  ((thread
    :initform nil
    :documentation "The internal thread object of the lisp implementation.")
   (parent-task
    :initform nil
    :reader parent-task
    :documentation "The parent task.")
   (child-tasks
    :initform (list)
    :accessor child-tasks
    :documentation "The list of child tasks.")
   (status
    :reader status
    :documentation "Fluent indicating the task status.")
   (unblocked
    :initform (make-fluent :name (gensym "TASK-UNBLOCKED-") :value t)
    :documentation "For internal use. nil when the thread is blocked.")
   (result
    :initform nil
    :documentation "The result of the task. When it terminates
		    normally, the slot contains the list of return
		    values. For evaporated tasks, it contains nil and
		    for failed tasks, it contains the condition
		    object.")
   (thread-fun
    :initform nil
    :initarg :thread-fun
    :documentation "The function of the task.")
   (path
    :initform *current-path*
    :initarg :path
    :reader task-path
    :documentation "The path of the task.")
   (code
    :initform (task-tree-node-code *current-task-tree-node*)
    :reader task-code
    :documentation "The code of the path.")
   (constraints
    :initform (list)
    :accessor task-constraints
    :documentation "The list of ordering constraint fluents. The task
		    is not allowed to start running before all fluents
		    are true.")))

(defun task-body (task thread-fun)
  (unwind-protect
       (with-slots (status parent-task constraints lexical-environment) task
	 (let ((*current-task* task))
	   (declare (special *current-task*))
	   (flet ((task-error-handler (condition)
		    (restart-case
			(invoke-debugger condition)
		      (terminate-task ()
			:report "Terminate failed task."
			(terminate task
				   :failed (make-condition 'rethrown-error
							   :error condition))))))
	     (handler-bind
		 ((rethrown-error
		   (lambda (condition)
		     (terminate task :failed condition)))
		  (plan-error
		   (lambda (condition)
		     (if *break-on-plan-failures*
			 (task-error-handler condition)
			 (terminate task
				    :failed (make-condition 'rethrown-error
							    :error condition)))))
		  (error #'task-error-handler))
	       (setf (value status) :running)
	       (wait-for (apply #'fl-and constraints))
	       (let ((result (multiple-value-list (funcall thread-fun))))
		 (terminate task :done result))))))
    (loop for child in (child-tasks task) do
      (terminate child :evaporated))))

(defun task-terminate (task status result)
  (unless (value (task-dead task))
    (setf (slot-value task 'result) result
	  (value (status task)) status)))

(defmethod initialize-instance :after
    ((task task) &key (thread-fun nil)
		      (ignore-no-parent nil)
		      (run-thread t) &allow-other-keys)
    (when *save-tasks*
      (as-atomic-operation
	(push task *tasks*)))
    (with-slots (status lexical-environment) task
      (setf status (make-fluent :name (gensym "TASK-STATUS-") :value :created))
      (when (and thread-fun run-thread)
	(execute task
		 :thread-fun thread-fun
		 :ignore-no-parent ignore-no-parent))))

(defmethod execute
    ((task task) &key (thread-fun (slot-value task 'thread-fun) thread-fun?)
     (ignore-no-parent nil))
  (assert (not (null thread-fun)))
  (when thread-fun?
    (setf (slot-value task 'thread-fun) thread-fun))
  (terminate-protect
    (let* ((lock (make-lock))
	   (thread-barrier (make-condition-variable :lock lock))
	   (barrier-pulsed nil)
	   (name (gensym "TASK-"))
	   (task-running (not (eq (status task) :created)))
	   (task-tree *task-tree*)
	   (current-node *current-task-tree-node*))
      ;; Register the task in the corresponding node.
      (flet ((thread-fun ()
	       (block nil
		 (flet ((do-terminate (status result)
			  (without-termination
			    (task-terminate task status result)
			    (return result))))
		   (with-termination-handler #'do-terminate
		     (unwind-protect
			  (progn
			    (with-lock-held (lock)
			      (unless barrier-pulsed
				(condition-variable-wait thread-barrier)))
			    (unless (value (task-dead task))
			      (let ((*current-task* task)
				    (*current-path* (task-path task))
				    (*task-tree* task-tree)
				    (*current-task-tree-node* current-node))
				(declare (special *current-task
						  *current-path*
						  *task-tree*
						  *current-task-tree-node*))
				(task-body task thread-fun))
			      (warn "task body terminated.")))
		       (unless (value (task-dead task))
			 (error "unhanlded exit ~a ~a" task (status task)))))))))
	(unless (or *current-task* ignore-no-parent)
	  (error "*current-task* unbound."))
	(with-slots (thread result parent-task) task
	  (when thread
	    (error "Task already runs a thread."))
	  ;; The barrier assures that the thread slot is bound when
	  ;; the task-body runs.
	  (unless (value (task-dead task))
	    (when *current-task*
	      (register-child *current-task* task))
	    (setf parent-task *current-task*)
	    (unwind-protect
		 (setf thread (spawn-thread name #'thread-fun))
	      (with-lock-held (lock)
		(setf barrier-pulsed t)
		(condition-variable-signal thread-barrier)))
	    (wait-for task-running)))))))

(defmethod terminate ((task task) task-status &optional (result-value nil))
  (if (eq *current-task* task)
      (signal-terminate task-status result-value)
      (terminate-protect
	(with-slots (thread status) task
	  (cond (thread
		 ;; We want to ignore termination errors. They may
		 ;; happen when the thread already died. Checking for
		 ;; a running thread does not help because it can die
		 ;; between the check and termination.
		 (ignore-errors
		   (run-in-thread thread
		     (lambda ()
		       (unless (signal-terminate task-status result-value)
			 ;; something bad has happened. Our signal
			 ;; handler is not yet set up.
			 (task-terminate task task-status result-value))))))
		(t
		 (unless (value (task-dead task))
		   (setf (value (status task)) task-status))))))))

(defmethod executed ((task task))
  (not (null (slot-value task 'thread))))

(defmethod result ((task task))
  (with-slots (result) task
    (if (listp result)
	(apply #'values result)
	result)))

(defmethod register-child ((task task) child)
  (pushnew child (slot-value task 'child-tasks))
  task)

(defmethod suspend ((task task))
  (with-slots (thread unblocked status) task
    (flet ((suspend-function ()
	     (wait-for unblocked :wait-status :blocked)
	     t))
      (wait-for (not (eq status :created)))
      (loop for child in (child-tasks task) do (suspend child))
      (ignore-errors
	(run-in-thread thread
	  (lambda ()
	    (when (member (value (status task)) '(:running :waiting))
	      (setf (value unblocked) nil)
	      (signal-suspend #'suspend-function))))))))

(defmethod wake-up ((task task))
  (setf (value (slot-value task 'unblocked)) t))

(defmethod join-task ((task task))
  (when (eq task *current-task*)
    (error "Cannot join current task."))
  (wait-for (task-dead task))
  (if (eq (value (status task)) :failed)
      (fail (result task))
      (result task)))
