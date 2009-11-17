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

(defvar *fact-groups* (make-hash-table :test 'eq))

(defun fact-head (fact)
  (car fact))

(defun fact-clauses (fact)
  (cdr fact))

(defmacro <- (fact &body fact-code)
  (declare (ignore fact fact-code))
  `(error "Facts cannot be declared outside fact-groups."))

(defun create-fact-group (name)
  "Returns a new factgroup named name."
  (declare (ignore name))
  nil)

(defun add-fact-group (name fact-group)
  "Adds a fact-group or overwrites one with the same name."
  (setf (gethash name *fact-groups*)
        fact-group))

(defun extend-fact-group (name additional-fact-group)
  "Extends fact-group with name by the facts in
  additional-fact-group. The additional facts are put after the
  already known facts."
  (setf (gethash name *fact-groups*)
        (nconc (gethash name *fact-groups*)
               additional-fact-group)))

(defun add-fact (fact-group fact-pattern fact-body)
  "Destructively adds a fact to a fact-group. The fact-pattern and
   body are not modified."
  (nconc fact-group `((,fact-pattern . ,fact-body))))

(defmacro def-fact-group (fact-group-name &body facts)
  `(macrolet ((<- (fact &body fact-code)
                `(setf fact-group (add-fact fact-group ',fact ',fact-code))))
     (let ((fact-group (create-fact-group ',fact-group-name)))
       ,@facts
       (add-fact-group ',fact-group-name fact-group))))

(defun find-fact-if (predicate)
  "Searches for a fact matching the predicate and returns it. nil
   otherwise."
  (loop for factgroup being the hash-values of *fact-groups*
     for found = (find-if predicate factgroup :key #'car)
     when found do (return found)))

(defun find-facts-if (predicate)
  "Find all facts matching the predicate."
  (loop for factgroup being the hash-values of *fact-groups*
     for found = (loop for fact in factgroup
                    when (funcall predicate (car fact)) collecting fact)
     when found nconcing found))

(defun map-facts (fun)
  "Apply function to every fact and collect the results of fun."
  (loop for factgroup being the hash-values of *fact-groups*
       nconcing (loop for fact in factgroup
                     collecting (funcall fun (car fact)))))

(defun reduce-fact-definitions (fun &optional (initial-value nil))
  "Like reduce but for facts with one difference. An initial-value is
   always used instead of using the first fact if no initial-value is passed."
  (labels ((%reduce-facts (last-result facts)
             (reduce fun facts :initial-value last-result))
           (%reduce-factgroups (last-result)
             (loop
                with new-result = last-result
                for factgroup being the hash-values of *fact-groups*
                do (setf new-result  (%reduce-facts new-result factgroup))
                finally (return new-result))))
    (%reduce-factgroups initial-value)))