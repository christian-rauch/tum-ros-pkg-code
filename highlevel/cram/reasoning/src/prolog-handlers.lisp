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

(in-package :crs)

(defvar *prolog-handlers* (make-hash-table :test 'eq))

(defmacro def-prolog-handler (name (bdgs &rest pattern) &body body)
  `(setf (gethash ',name *prolog-handlers*)
         (lambda (,bdgs ,@pattern)
           ,@body)))

(defun get-prolog-handler (name)
  (gethash name *prolog-handlers*))

(def-prolog-handler and (bdgs &rest pats)
  (labels ((do-and (clauses bdgs)
             (if clauses
                 (lazy-mapcan (lambda (goal-1-bdgs)
                                (do-and (cdr clauses) goal-1-bdgs))
                              (prolog (car clauses) bdgs))
                 (list bdgs))))
    (do-and pats bdgs)))

(def-prolog-handler or (bdgs &rest pats)
  (lazy-mapcan (lambda (pat)
                 (let ((result (prolog pat bdgs)))
                   result))
               pats))

(def-prolog-handler not (bdgs form)
  (unless (prolog form bdgs)
    (list bdgs)))

(def-prolog-handler lisp-fun (bdgs function &rest args)
  (let ((result (apply (symbol-function function)
                       (mapcar (rcurry #'var-value bdgs) (butlast args))))
        (result-var (car (last args))))
    ;; (format t "result: ~a ~a bdgs: ~a~%" result result-var bdgs)
    (multiple-value-bind (new-bdgs matched?) (unify result-var result bdgs)
      (when matched?
        (list new-bdgs)))))

(def-prolog-handler lisp-pred (bdgs pred &rest args)
  (when (apply (symbol-function pred)
               (mapcar (rcurry #'var-value bdgs) args))
    (list bdgs)))

(def-prolog-handler is-bound (bdgs var-name)
  (unless (and (is-var var-name)
               (is-var (var-value var-name bdgs)))
    (list bdgs)))

(def-prolog-handler fail (bdgs)
  (declare (ignore bdgs))
  nil)
