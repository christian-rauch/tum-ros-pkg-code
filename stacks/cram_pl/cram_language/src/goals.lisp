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

(defun matching-goal-fun (name args &optional (goal-funs (get name :goals)))
  (when goal-funs
    (multiple-value-bind (bdgs ok?) (cut:pat-match (caar goal-funs) args)
      (if ok?
          (values (car goal-funs) bdgs)
          (matching-goal-fun name args (cdr goal-funs))))))

(defun make-goal-fun (name pattern body)
  (with-gensyms (bdgs-var)
    (let* ((body `(with-tags ,@body))
           (goal-fun `(lambda (,bdgs-var)
                        (with-task-tree-node (:path-part `(goal (,',name ,',@pattern))
                                              :name ,(format nil "GOAL-~a" name)
                                              :sexp `(,',name ,',@pattern ,',body)
                                              :lambda-list ,(cut:vars-in pattern)
                                              :parameters (mapcar (alexandria:rcurry #'cut:var-value ,bdgs-var)
                                                                  ',(cut:vars-in pattern)))
                          ,body))))
      goal-fun)))

(defun register-goal (name pattern goal-fun)
  (when (assoc pattern (get name :goals) :test #'equal)
    #+sbcl (sb-int:style-warn "Redifining goal `~a'" `(,name ,@pattern)))
  (cond ((not (get name :goals))
         (setf (get name :goals)
               (list (cons pattern goal-fun))))
        (t
         (setf (get-alist pattern (get name :goals) :test #'equal)
               goal-fun))))

(defun call-goal (name args)
  (multiple-value-bind (def bdgs) (matching-goal-fun name args)
    (unless def
      (error "No goal found that matches arguments."))
    (funcall (cdr def) bdgs)))

(defmacro declare-goal (name lambda-list &body body)
  (with-gensyms (pattern)
    `(progn
       (setf (get ',name :goals) nil)
       (defun ,name (&rest ,pattern)
         (block nil
           (apply (lambda ,lambda-list ,@body) ,pattern)
           (call-goal ',name ,pattern))))))

(defmacro def-goal ((name &rest pattern) &body body)
  "
Defines a new goal. Goals always have the form

\(<name> [pattern]*)

where patterns can be arbitrary expressions containing
variables (indicated by a ?-prefix) and symbols.
Example: (achieve (loc ?obj ?location)) When defining goals with
similar expressions (e.g. (loc Robot ?l) and (loc ?a ?b)) the most
specific expression must be defined first. Otherwise, the previously
defined expression will be overwritten.
"
  `(let ((goal-fun ,(make-goal-fun name pattern body)))
     (register-goal ',name ',pattern goal-fun)))

