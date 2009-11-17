;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Piotr Esden-Tempski <esdentem@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
