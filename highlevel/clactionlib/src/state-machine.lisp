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

(in-package :actionlib)

(defvar statusmap (mapcar (lambda (c) (cons (cdr c) (car c)))
                          (roslisp-msg-protocol:symbol-codes '<GOALSTATUS>)))

(defvar transitions '((:waiting-for-goal-ack
                       ((:pending    :pending)
                        (:active     :active)
                        (:rejected   :waiting-for-result)
                        (:recalling  :recalling)
                        (:recalled   :waiting-for-result)
                        (:preempted  :waiting-for-result)
                        (:succeeded  :waiting-for-result)
                        (:aborted    :waiting-for-result)
                        (:preempting :preempting)))
                      (:pending
                       ((:pending    :no-transition)
                        (:active     :active)
                        (:rejected   :waiting-for-result)
                        (:recalling  :recalling)
                        (:recalled   :waiting-for-result)
                        (:preempted  :waiting-for-result)
                        (:succeeded  :waiting-for-result)
                        (:aborted    :waiting-for-result)
                        (:preempting :preempting)))
                      (:active
                       ((:pending    :invalid-transition)
                        (:active     :no-transition)
                        (:rejected   :invalid-transition)
                        (:recalling  :invalid-transition)
                        (:recalled   :invalid-transition)
                        (:preempted  :waiting-for-result)
                        (:succeeded  :waiting-for-result)
                        (:preempting :preempting)))
                      (:waiting-for-result
                       ((:pending    :invalid-transition)
                        (:active     :no-transition)
                        (:rejected   :no-transition)
                        (:recalling  :invalid-transition)
                        (:recalled   :no-transition)
                        (:preempted  :no-transition)
                        (:succeeded  :no-transition)
                        (:aborted    :no-transition)
                        (:preempting :invalid-transition)))
                      (:waiting-for-cancel-ack
                       ((:pending    :no-transition)
                        (:active     :no-transition)
                        (:rejected   :waiting-for-result)
                        (:recalling  :recalling)
                        (:recalled   :waiting-for-result)
                        (:preempted  :waiting-for-result)
                        (:succeeded  :waiting-for-result)
                        (:aborted    :waiting-for-result)
                        (:preempting :preempting)))
                      (:recalling
                       ((:pending    :invalid-transition)
                        (:active     :invalid-transition)
                        (:rejected   :waiting-for-result)
                        (:recalling  :no-transition)
                        (:recalled   :waiting-for-result)
                        (:preempted  :waiting-for-result)
                        (:succeeded  :waiting-for-result)
                        (:aborted    :waiting-for-result)
                        (:preempting :preempting)))
                      (:preempting
                       ((:pending    :invalid-transition)
                        (:active     :invalid-tarnsition)
                        (:rejected   :invalid-transition)
                        (:recalling  :invalid-transition)
                        (:recalled   :invalid-transition)
                        (:preempted  :waiting-for-result)
                        (:succeeded  :waiting-for-result)
                        (:aborted    :waiting-for-result)
                        (:preempting :no-transition)))
                      (:done
                       ((:pending    :invalid-transition)
                        (:active     :invalid-transition)
                        (:rejected   :no-transition)
                        (:recalling  :invalid-transition)
                        (:recalled   :no-transition)
                        (:preempted  :no-transition)
                        (:succeeded  :no-transition)
                        (:aborted    :no-transition)
                        (:preempting :invalid-transition)))))

(defclass comm-state-machine ()
  ((action-goal
    :accessor action-goal
    :initarg :action-goal)
   (transition-cb
    :accessor transition-cb
    :type function
    :initarg :transition-cb)
   (feedback-cb
    :accessor feedback-cb
    :type function
    :initarg :feedback-cb)
   (send-goal-fn
    :accessor send-goal-fn
    :type function
    :initarg :send-goal-fn)
   (send-cancel-fn
    :accessor send-cancel-fn
    :type function
    :initarg :send-cancel-fn)
   (state
    :accessor state
    :initform :waiting-for-goal-ack)
   (mutex
    :accessor mutex
    :initform (portable-threads:make-lock))
   (latest-goal-status
    :accessor latest-goal-status
    :initform :pending)
   (latest-result
    :accessor latest-result
    :initform nil)))

(defgeneric update-status (self status-array))
(defgeneric transition-to (self state))
(defgeneric mark-as-lost (self))
(defgeneric update-result (self action-result))
(defgeneric update-feedback (self action-feedback))

(defun find-status-by-goal-id (status-array id)
  (find-if (lambda (s)
             (equal (id-val
                     (goal_id-val s))
                    id))
           (status_list-val status-array)))

(defun status-to-key (status)
  (cdr (assoc (status-val status) statusmap)))

(defun build-key (name)
  (intern (string-upcase (format nil "~a" name)) "KEYWORD"))

(defun ensure-is-key (key)
  (cond
    ((typep key 'keyword)
      key)
    ((typep key '(simple-array character *))
      (build-key key))
    (t
      (ros-error (roslisp actionlib ensure-is-key) "Ensure key failed because ~a is of type ~a" key (type-of key)))))

(defun find-car (item sequence)
  (find-if (lambda (i) (eq (car i) item)) sequence))

(defun get-transition (state &optional status)
  (if status
    (find-car (ensure-is-key status)
              (cadr (find-car (ensure-is-key state)
                              transitions)))
    (find-car (ensure-is-key state)
              transitions)))

(defmethod update-status ((self comm-state-machine) status-array)
  (portable-threads:with-lock-held ((mutex self))
    (let ((status (find-status-by-goal-id status-array
                                          (id-val
                                           (meth-in-ext-call 'goal_id-val
                                                             (action-goal self))))))
      (if (not status)
        (unless (find (state self) '(:waiting-for-goal-ack
                                     :waiting-for-result
                                     :done))
          (mark-as-lost self))
        (progn

          (setf (latest-goal-status self) (status-to-key status))

          (cond
            ((not (get-transition (state self)))
              (ros-error (roslisp actionlib)
                         "Rincewind meets DEATH because of a funny state ~a"
                         (state self)))
            ((not (get-transition (state self) (status-to-key status)))
              (ros-error (roslisp actionlib)
                         "Got an unknown status from the ActionServer: ~a"
                         (status-to-key status)))
            (t
              (let ( (next-transition (cadr (get-transition (state self) (status-to-key status)))))
                (cond
                  ((eq next-transition :no-transition)
                   nil)
                  ((eq next-transition :invalid-transition)
                    (ros-error (roslisp actionlib)
                               "Invalid goal status transition from ~a to ~a"
                               (state self)
                               (status-to-key status)))
                  (t
                    (transition-to self next-transition)))))))))))

(defmethod transition-to ((self comm-state-machine) state)
  (ros-debug (roslisp actionlib)
                     "Transitioning from ~a to ~a"
                     (state self)
                     state)
  (setf (state self) state)
  (when (transition-cb self)
    (funcall (transition-cb self) (make-instance 'goal-handle :comm-state-machine self))))

(defmethod mark-as-lost ((self comm-state-machine))
  (setf (slot-value self 'latest-goal-status) :lost)
  (transition-to self :done))

(defmethod update-result ((self comm-state-machine) action-result)
  (when (equal (id-val
                (meth-in-ext-call 'goal_id-val
                                  (action-goal self)))
               (id-val
                (meth-in-ext-call 'goal_id-val
                                  (meth-in-ext-call 'status-val
                                                    action-result))))
    (portable-threads:with-lock-held ((slot-value self 'mutex))
      (setf (latest-goal-status self) (status-to-key (meth-in-ext-call 'status-val action-result)))
      (setf (latest-result self) action-result)
      (cond
        ((find (state self) '(:waiting-for-goal-ack
                              :pending
                              :active
                              :waiting-for-result
                              :recalling
                              :preempting
                              :waiting-for-cancel-ack))
          (transition-to self :done))
        ((eq (state self) :done)
          (ros-error (roslisp actionlib) "Got a result while already being in DONE state"))
        (t
          (ros-error (roslisp actionlib) "Rincewind fell off the disc while casting ~a state." (state self)))))))

(defmethod update-feedback ((self comm-state-machine) action-feedback)
  (when (and
         (feedback-cb self)
         (equal (id-val
                 (meth-in-ext-call 'goal_id-val
                                   (action-goal self)))
                (id-val
                 (meth-in-ext-call 'goal_id-val
                                   (meth-in-ext-call 'status-val
                                                     action-feedback)))))
    (funcall (feedback-cb self)
           action-feedback)))