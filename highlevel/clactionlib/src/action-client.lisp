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

(defclass action-client ()
  ((ns
    :accessor ns
    :initarg :ns)
   (action-spec
    :accessor action-spec
    :initarg :action-spec)
   (action-goal
    :accessor action-goal)
   (action-result
    :accessor action-result)
   (action-feedback
    :accessor action-feedback)
   (pub-goal
    :accessor pub-goal)
   (pub-cancel
    :accessor pub-cancel)
   (manager
    :accessor manager)))

(defmethod initialize-instance :after ((self action-client) &key)
  (let ( (action-spec-inst (make-instance (action-spec self)))
         (action-spec-package (symbol-package (action-spec self))) )

    (setf (action-goal self) (type-of (slot-value action-spec-inst (intern "ACTION_GOAL" action-spec-package))))
    (setf (action-result self) (type-of (slot-value action-spec-inst (intern "ACTION_RESULT" action-spec-package))))
    (setf (action-feedback self) (type-of (slot-value action-spec-inst (intern "ACTION_FEEDBACK" action-spec-package))))

    (setf (pub-goal self) (roslisp:advertise (format nil "~a/goal" (ns self)) (action-goal self)))
    (setf (pub-cancel self) (roslisp:advertise (format nil "~a/cancel" (ns self)) '<GOALID>))

    (setf (manager self) (make-instance 'goal-manager :action-spec (action-spec self)))
    (setf (send-goal-fn (manager self)) (lambda (msg) (publish (pub-goal self) msg)))
    (setf (send-cancel-fn (manager self)) (lambda (msg) (publish (pub-cancel self) msg)))

    (subscribe (format nil "~a/status" (ns self)) '<GOALSTATUSARRAY> (lambda (msg) (ac-status-cb self msg)))
    (subscribe (format nil "~a/result" (ns self)) (action-result self) (lambda (msg) (ac-result-cb self msg)))
    (subscribe (format nil "~a/feedback" (ns self)) (action-feedback self) (lambda (msg) (ac-feedback-cb self msg)))))

(defgeneric send-goal (self goal &key transition-cb feedback-cb))
(defgeneric cancel-all-goals (self))
(defgeneric ac-status-cb (self msg))
(defgeneric ac-result-cb (self msg))
(defgeneric ac-feedback-cb (self msg))

(defmethod send-goal ((self action-client) goal &key (transition-cb nil) (feedback-cb nil))
  (init-goal (manager self) goal :transition-cb transition-cb :feedback-cb feedback-cb))

(defmethod cancel-all-goals ((self action-client))
  (publish (pub-cancel self) (make-instance '<GOALID>
                               :stamp 0
                               :id 0)))

(defmethod ac-status-cb ((self action-client) msg)
  (update-statuses (manager self) msg))

(defmethod ac-result-cb ((self action-client) msg)
  (update-results (manager self) msg))

(defmethod ac-feedback-cb ((self action-client) msg)
  (update-feedbacks (manager self) msg))
