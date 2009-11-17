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

(defmacro with-parallel-childs ((running done failed) child-forms &body watcher-body)
  "Execute child-forms in parallel and execute watcher-body whenever
   any child changes its status. The variables running, done and
   failed are lexically bound in watcher-body and are lists of all
   running, done and failed tasks. Please note that watcher-body can
   be terminated by a return call."
  `(with-task
     ,@(mapcar (lambda (form)
                 `(make-instance 'task :thread-fun (lambda () ,form)))
               child-forms)
     (block nil
       (whenever ((apply #'pulsed (mapcar #'status (child-tasks *current-task*))))
         (multiple-value-bind (,running ,done ,failed)
             (loop for task in (child-tasks *current-task*)
                when (member (value (status task)) '(:running :blocked :waiting :created))
                collect task into running
                when (member (value (status task)) '(:done :evaporated))
                collect task into done
                when (eq (value (status task)) :failed)
                collect task into failed
                finally (return (values running done failed)))
           ,@watcher-body)))))

(defun get-alist (name alist &key key test test-not)
  (let ((result (apply #'assoc `(,name ,alist
                                       ,@(when key `(:key ,key))
                                       ,@(when test `(:test ,test))
                                       ,@(when test-not `(:test-not ,test-not))))))
    (if result
        (values (cdr result) t)
        (values nil nil))))

(defun (setf get-alist) (new-value name alist &key key test test-not)
  (let ((result (apply #'assoc `(,name ,alist
                                       ,@(when key `(:key ,key))
                                       ,@(when test `(:test ,test))
                                       ,@(when test-not `(:test-not ,test-not))))))
    (if result
        (setf (cdr result) new-value)
        (let ((last-elem (last alist)))
          (setf (cdr last-elem) (list (cons name new-value))))))
  new-value)
