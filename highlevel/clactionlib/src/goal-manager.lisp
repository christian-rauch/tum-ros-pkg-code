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

(defclass goal-manager ()
  ((list-mutex
    :accessor list-mutex
    :initform (portable-threads:make-lock))
   (statuses
    :accessor statuses
    :initform nil)
   (action-spec
    :accessor action-spec
    :initarg :action-spec)
   (action-goal
    :accessor action-goal)
   (action-result
    :accessor action-result)
   (action-feedback
    :accessor action-feedback)
   (send-goal-fn
    :accessor send-goal-fn)
   (send-cancel-fn
    :accessor send-cancel-fn)))

(defmethod initialize-instance :after ((self goal-manager) &key)
  (let ( (action-spec-inst (make-instance (action-spec self)))
         (action-spec-package (symbol-package (action-spec self))) )

    (setf (action-goal self) (type-of (slot-value action-spec-inst (intern "ACTION_GOAL" action-spec-package))))
    (setf (action-result self) (type-of (slot-value action-spec-inst (intern "ACTION_RESULT" action-spec-package))))
    (setf (action-feedback self) (type-of (slot-value action-spec-inst (intern "ACTION_FEEDBACK" action-spec-package))))))

(defun generate-ros-id ()
  (let ((now (ros-time)))
    (make-instance '<GOALID> :stamp now :id (format nil "~a-~a" now (get-internal-real-time)))))

(defgeneric init-goal (self goal &key transition-cb feedback-cb))
(defgeneric get-live-statuses (self))
(defgeneric update-statuses (self status-array))
(defgeneric update-results (self action-result))
(defgeneric update-feedbacks (self action-feedback))

(defmethod init-goal ((self goal-manager) goal &key transition-cb feedback-cb)
  (let* ( (action-goal (make-instance (action-goal self)
                         :header (make-instance '<HEADER>)
                         :goal_id (generate-ros-id)
                         :goal goal))
          (csm (make-instance 'comm-state-machine
                 :action-goal action-goal
                 :transition-cb transition-cb
                 :feedback-cb feedback-cb
                 :send-goal-fn (send-goal-fn self)
                 :send-cancel-fn (send-cancel-fn self))) )

    (portable-threads:with-lock-held ((list-mutex self))
      (setf (statuses self) (append (statuses self) (list (tg:make-weak-pointer csm)))))

    (funcall (send-goal-fn self) action-goal)
    (make-instance 'goal-handle :comm-state-machine csm)))

(defmethod get-live-statuses ((self goal-manager))
  (portable-threads:with-lock-held ((list-mutex self))
    (mapcan (lambda (x)
              (let ( (val (sb-ext:weak-pointer-value x)) )
                (when val
                  (list val))))
            (statuses self))))

(defmethod update-statuses ((self goal-manager) status-array)
  (portable-threads:with-lock-held ((list-mutex self))
    (setf (statuses self) (mapcan (lambda (x)
                                    (let ( (val (sb-ext:weak-pointer-value x)) )
                                      (when val
                                        (list x)))) (statuses self))))
  (mapcar (lambda (x) (update-status x status-array)) (get-live-statuses self)))

(defmethod update-results ((self goal-manager) action-result)
  (mapcar (lambda (x) (update-result x action-result)) (get-live-statuses self)))

(defmethod update-feedbacks ((self goal-manager) action-feedback)
  (mapcar (lambda (x) (update-feedback x action-feedback)) (get-live-statuses self)))