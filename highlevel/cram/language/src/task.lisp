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

(defvar *current-task* nil "Dynamically bound current task.")

(defvar *save-tasks* nil "When t, every created task is pushed to *tasks*.")
(defvar *tasks* (list) "List of all created tasks. Used for debugging")

;;; Protocol definition of task
(defgeneric execute (task &key thread-fun ignore-no-parent)
  (:documentation
   "When the task has been instantiated without a thread function or
    the function has not been executed yet, it can be executed with
    this method.  When `ignore-no-parent' is true, no error message is
    signaled when *CURRENT-TASK* is unbound."))

(defgeneric executed (task)
  (:documentation
   "Returns NIL if the task has not been executed yet."))

(defgeneric result (task)
  (:documentation
   "Returns the result of the thread. For multiple values, this
    function also returns multiple values."))

(deftype task-status ()
  `(:created :running :blocked :waiting :done :evaporated :failed))

(defgeneric status (task)
  (:documentation "Returns the status fluent of the task.")
  (declare (values task-status)))

(defgeneric child-tasks (task)
  (:documentation
   "Returns the (currently unsorted) list of child tasks."))

(defgeneric parent-task (task)
  (:documentation
   "Returns the parent task."))

(defgeneric register-child (task child)
  (:documentation
   "Registers a child task. It is evaporated when the parent task dies."))

(defgeneric terminate (task task-status &optional result-value)
  (:documentation
   "Terminates a task. The status' fluent and the result value are set
    and the task function is terminated.

    TERMINATE does nothing if the thread is running, blocked or
    waiting."))

(defgeneric suspend (task)
  (:documentation
   "Blocks a thread until it is terminated or awaken."))

(defgeneric wake-up (task)
  (:documentation
   "Wakes up a blocked thread. If the thread is not blocked,
    it does nothing."))

(defgeneric join-task (task)
  (:documentation
   "Waits for a task to finish. When it fails, this function rethrows
    the failure, when it succeeds, it returns the result of the
    task."))

(defgeneric task-path (task)
  (:documentation
   "Returns the path of the task."))

(defmacro with-status ((status &optional (task '*current-task*)) &body body)
  "Sets the task's status to `status' before executing body, and
   restores the old value at the end. The status is only restored to
   its original value if the thread status is still `status' upon
   executing the cleanup form."
  (with-gensyms (old-status)
    (once-only (task)
      `(let ((,old-status (and ,task (value (status ,task)))))
	 (unwind-protect
	      (progn
		(when ,task
		  (setf (value (status ,task)) ,status))
		,@body)
	   (when (and ,task (eq (value (status ,task)) ,status))
	     (setf (value (slot-value ,task 'status)) ,old-status)))))))

(defun clear-saved-tasks ()
  (setf *tasks* (list)))

(defun task-alive (task)
  "Returns a fluent indicating if the task is alive"
  (fl-funcall #'member (status task) '(:created :running :waiting :blocked)))

(defun task-dead (task)
  (fl-funcall #'member (status task) '(:failed :done :evaporated)))

(define-condition suspend-notification (condition)
  ((suspend-handler :initform (required-argument :handler)
		    :initarg :handler
		    :reader suspend-handler))
  (:documentation
   "This condition is signaled _in the contour_ of a task that is
    supposed to be blocked. By establishing a handler for this
    condition and by means of the following restarts, a task can
    control if and when blocking occurs.

      IGNORE-SUSPEND:
	the request for suspension should be ignored.

      SUSPEND:
	the suspension should proceed. (By explicitly invoking this
	restart, you can specify things that are supposed to happen
	before and after suspension.)

    If the condition is not handled, suspension proceeds implicitly.

    The suspension process is performed by invoking the function
    stored in the `suspension-handler' slot.

    You probably want to use one of the abstractions SUSPEND-PROTECT,
    ON-SUSPENSION, or WITHOUT-SUSPENSION."))

(defun signal-suspend (suspend-function)
  (restart-case (signal 'suspend-notification :handler suspend-function)
    (ignore-suspend ()
      (return-from signal-suspend t))
    (suspend ()))			; fall through
  (funcall suspend-function))

(defmacro suspend-protect (form &body protection-forms)
  "Executes `form'. When the current task is suspended while the form runs,
   `protection-forms' are executed, and the task is suspended. After
   wakeup, form is executed again.  WARNING: Do not perform side
   effects! The number of executions depends on the number of
   blocks/restarts!"
  (with-gensyms (retry)
    `(prog ()
	,retry
	(handler-case
	    (return (unwind-protect ,form ,@protection-forms))
	  (suspend-notification (condition)
	    ;; rethrow the condition to allow outside handlers to handle
	    (signal condition)
	    (funcall (suspend-handler condition))
	    (go ,retry))))))

(defmacro on-suspend (form &body when-suspended-forms)
  "Executes `when-suspended-forms' whenever a suspension signal occurs
   while `form' is being executed."
  `(handler-bind
       ((suspend-notification (lambda  (condition)
				(declare (ignore condition))
				,@when-suspended-forms)))
     ,form))

(defmacro without-suspend (&body body)
  "Ignore all suspension signals while `body' is running."
  `(handler-bind
       ((suspend-notification (lambda (condition)
				(declare (ignore condition))
				(invoke-restart 'ignore-suspend))))
     ,@body))

(define-condition termination-notification (condition)
  ((status :initform :evaporated :initarg :status :accessor status)
   (result :initform nil :initarg :result :accessor result))
  (:documentation
   "This condition is signaled _in the contour_ of a task that is
    supposed to be terminated. By establishing a handler for this
    condition and by means of the following restarts, a task can
    control if and when termnation is supposed to happen.

      IGNORE-TERMINATE:
	the request for termination should be ignored.

      TERMINATE: the suspension should proceed. (By explicitly
	invoking this restart, you can specify things that are
	supposed to happen before termination.)

    If the condition is not handled, termination proceeds implicitly.

    The suspension process is performed by invoking the function
    stored in the `suspension-handler' slot.

    You probably want to use one of the abstractions
    TERMINATE-PROTECT, ON-TERMINATION, or WITHOUT-TERMINATION."))

(defun signal-terminate (status &optional result)
  (restart-case
      (cond ((typep status 'condition)
	     (signal status))
	    (t
	     (signal (make-condition 'termination-notification
				     :status status :result result))))
    (ignore-terminate ()
      t)))

(defmacro with-termination-handler (handler &body body)
  "Executes handler whenever a termination signal occurs."
  (with-gensyms (handler-fun)
    `(let ((,handler-fun ,handler))
       (handler-bind
	    ((termination-notification (lambda (condition)
					 (funcall ,handler-fun
						  (status condition)
						  (result condition)))))
	  ,@body))))

(defmacro without-termination (&body body)
  "Ignors all terminations occuring while body is running"
  `(handler-bind
       ((termination-notification (lambda (condition)
				    (declare (ignore condition))
				    (invoke-restart 'ignore-terminate))))
     ,@body))

(defmacro terminate-protect (&body body)
  "Assures that body is finished before the termination request is
  processed."
  (with-gensyms (received-termination)
    `(let ((,received-termination nil))
       (handler-bind
	   ((termination-notification (lambda (condition)
					(setf ,received-termination condition)
					(invoke-restart 'ignore-terminate))))
	 ,@body)
       (when ,received-termination
	 (signal-terminate ,received-termination)))))
