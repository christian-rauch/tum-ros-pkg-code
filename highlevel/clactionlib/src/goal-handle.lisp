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

(defclass goal-handle ()
  ((comm-state-machine
    :accessor comm-state-machine
    :initarg :comm-state-machine)))

(defgeneric reset (self))
(defgeneric expired? (self))
(defgeneric same? (self o))
(defgeneric cancel (self))
(defgeneric try-cancel (self)
  (:documentation "Cancels if not already done."))
(defgeneric get-comm-state (self))
(defgeneric get-goal-status (self))
(defgeneric get-result (self))
(defgeneric get-terminal-state (self))

(defmethod reset ((self goal-handle))
  (setf (comm-state-machine self) nil))

(defmethod expired? ((self goal-handle))
  (not (comm-state-machine self)))

(defmethod same? ((self goal-handle) (o goal-handle))
  (if (and (expired? self) (expired? o))
    t
    (equal (comm-state-machine self) (comm-state-machine o))))

(defmethod cancel ((self goal-handle))
  (if (expired? self)
    (roslisp:ros-error (roslisp actionlib) "Calling cancel on an inactive goal-handle")
    (portable-threads:with-lock-held ((mutex (comm-state-machine self)))
      (funcall (send-cancel-fn (comm-state-machine self)) (make-instance '<GOALID>
                                                          :id (id-val
                                                               (meth-in-ext-call 'goal_id-val
                                                                                 (action-goal
                                                                                  (comm-state-machine self))))))
      (transition-to (comm-state-machine self) :waiting-for-cancel-ack))))

(defmethod try-cancel ((self goal-handle))
  (portable-threads:with-lock-held ((mutex (comm-state-machine self)))
    (unless (or (expired? self) (eq (state (comm-state-machine self))
                                    :done))
      (funcall (send-cancel-fn (comm-state-machine self)) (make-instance '<GOALID>
                                                            :id (id-val
                                                                 (meth-in-ext-call 'goal_id-val
                                                                                   (action-goal
                                                                                    (comm-state-machine self))))))
      (transition-to (comm-state-machine self) :waiting-for-cancel-ack))))

(defmethod get-comm-state ((self goal-handle))
  (if (not (comm-state-machine self))
    (progn
      (ros-error (roslisp actionlib) "Trying to get-comm-state on an inactive goal-handle.")
      :lost)
    (state (comm-state-machine self))))

(defmethod get-goal-status ((self goal-handle))
  (if (not (comm-state-machine self))
    (progn
      (ros-error (roslisp actionlib) "Trying to get-goal-status on an inactive goal-handle.")
      :pending)
    (latest-goal-status (comm-state-machine self))))

(defmethod get-result ((self goal-handle))
  (if (not (comm-state-machine self))
    (progn
      (ros-error (roslisp actionlib) "Trying to get-result on an inactive goal-handle.")
      nil)
    (if (latest-result (comm-state-machine self))
      (meth-in-ext-call 'result-val (latest-result (comm-state-machine self)))
      (ros-error (roslisp actionlib) "Trying to get-result on a comm-state-machine which does not have a result!"))))

(defmethod get-terminal-state ((self goal-handle))
  (if (not (comm-state-machine self))
    (progn
      (ros-error (roslisp actionlib) "Trying to get-goal-status on an inactive goal-handle.")
      :lost)
    (portable-threads:with-lock-held ((mutex (comm-state-machine self)))
      (when (not (eq (state (comm-state-machine self)) :done))
        (ros-warn (roslisp actionlib) "Asking for the terminal state when we are in [~a]"
                  (state (comm-state-machine self))))

      (let ( (goal-status (get-goal-status self)) )
        (if (find goal-status '(:preempted
                                :succeeded
                                :aborted
                                :rejected
                                :recalled
                                :lost))
          goal-status
          (progn
            (ros-error (roslisp actionlib) "Asking for a terminal state, but the goal status is ~a" goal-status)
            :lost))))))