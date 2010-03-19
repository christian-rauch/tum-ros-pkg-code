;;;
;;; Copyright (C) 2009 by Piotr Esden-Tempski <esdentem@cs.tum.edu>
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

(in-package :kipla)

(defmacro def-actionlib-interface (name namespace action-spec)
  (let* ((name-str (symbol-name name))
         (init-name (intern (string-upcase (format nil "~a-init" name-str))))
         (cleanup-name (intern (string-upcase (format nil "~a-cleanup" name-str))))
         (get-ac-name (intern (string-upcase (format nil "~a-get-ac" name-str))))
         (get-handle-name (intern (string-upcase (format nil "~a-get-handle" name-str))))
         (send-goal-name (intern (string-upcase (format nil "~a-send-goal" name-str))))
         (cancel-goal-name (intern (string-upcase (format nil "~a-cancel" name-str)))))
    `(let ((ac nil)
           (handle nil)
           (canceled nil))

       (defun ,init-name ()
         (setf ac (make-instance 'actionlib:action-client
                     :ns ,namespace
                     :action-spec ,action-spec)))

       (defun ,cleanup-name ()
         (setf ac nil))

       (defun ,get-ac-name () ac)
       (defun ,get-handle-name () handle)

       (defun ,send-goal-name (goal &optional feedback-cb)
         (when handle
           (fail (make-condition 'multiple-goal-err
                                 :format-control "Trying to send a goal to manipulation although there is already a goal in process.")))
         (let ((finished (portable-threads:make-condition-variable))
               (result nil))
           (setf canceled nil)
           (unwind-protect
                (progn
                  (setf handle
                        (actionlib:send-goal ac goal
                                             :transition-cb (lambda (x)
                                                              (when (eq (actionlib:get-comm-state x) :done)
                                                                (cond ((and (eq (actionlib:get-goal-status x) :lost)
                                                                            (not canceled))
                                                                       (setf result
                                                                             (make-condition 'goal-is-lost-err
                                                                                             :format-control "A goal of ~a got lost."
                                                                                             :format-arguments '(,name-str))))
                                                                      (t
                                                                       (setf canceled t)
                                                                       (setf result (actionlib:get-result x))))
                                                                (portable-threads:with-lock-held (finished)
                                                                  (portable-threads:condition-variable-signal finished))))
                                             :feedback-cb feedback-cb))
                  (portable-threads:with-lock-held (finished)
                    (portable-threads:condition-variable-wait finished)))
             (unless result
               (log-msg :info "[~a action] trying to cancel." ,name-str)
               (setf canceled t)
               (actionlib:try-cancel handle))
             (setf handle nil))
           (when (typep result 'error)
             (error result))
           result))

       (defun ,cancel-goal-name ()
         (log-msg :info "[Manipulation action client] Canceling goal.")
         (unless handle
           (fail (make-condition 'invalid-cancel-err
                                 :format-control "Trying to cancel a goal while there is no goal being processed.")))
         (setf canceled t)
         (actionlib:cancel handle)))))
