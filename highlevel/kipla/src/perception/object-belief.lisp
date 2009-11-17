;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Lorenz Moesenlechner <moesenle@cs.tum.edu>
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

(in-package :kipla-reasoning)

(defstruct perceived-object
  lo-id
  properties
  probability
  desig
  (timestamp (current-timestamp)))

;;; For now, the solution for belief state management might not be the
;;; best one. It does not take into account probabilities, does not
;;; assert facts, ...

;;; We keep a list of perceived-object structs. Whenever the system
;;; decides that two objects are the same, it updates the
;;; corresponding perceived-object instance and returns it. Otherwise,
;;; the new instance is registered.

(defvar *perceived-objects* nil "List of perceived objects")

(defvar *incompatible-properties*
  '(("Jug" "Mug" "IceTea")
    ("black" "red"))
  "List of cop properties that are incompatible, i.e. they describe
  different objects.")

(defun clear-object-belief ()
  (setf *perceived-objects* nil))

(defmethod update-belief ((obj perceived-object) &optional reference-object)
  (let ((known-object (or reference-object
                          (find-if (alexandria:curry #'perceived-objects-equal? obj)
                                   *perceived-objects*))))
    (cond (known-object
           (kipla:log-msg :info "[update-belief] updating object.")
           (update-perceived-object known-object obj))
          (t
           (kipla:log-msg :info "[update-belief] registerung new object.")
           (push obj *perceived-objects*)
           obj))))

(defun compatible-properties (props-1 props-2)
  (or (null props-1)
      (let* ((matching-prop-1 (car props-1))
             (matching-prop-set (find-if (lambda (p)
                                           (member matching-prop-1 p :test #'equal))
                                         *incompatible-properties*))
             (matching-prop-2 (find-if (alexandria:rcurry #'member matching-prop-set
                                                          :test #'equal)
                                       props-2)))
        (when (or (null matching-prop-2)
                  (equal matching-prop-1 matching-prop-2))
          (compatible-properties (cdr props-1) props-2)))))

(defun perceived-objects-equal? (obj-1 obj-2)
  (and (compatible-properties (perceived-object-properties obj-1)
                              (perceived-object-properties obj-2))
       (or (jlo:inlier? (perceived-object-lo-id obj-1)
                       (perceived-object-lo-id obj-2))
           (jlo:inlier? (perceived-object-lo-id obj-2)
                       (perceived-object-lo-id obj-1)))))

(defun update-perceived-object (old new)
  ;; This is a bad hack. We just use the new lo-id, build the union of
  ;; properties, use the new probability and new timestampt and keep
  ;; the old designator. Todo: good solution.
  (setf (perceived-object-lo-id old)
        (perceived-object-lo-id new))
  (setf (perceived-object-properties old)
        (union (perceived-object-properties old)
               (perceived-object-properties new)
               :test #'equal))
  (setf (perceived-object-probability old)
        (perceived-object-probability new))
  (setf (perceived-object-timestamp old)
        (perceived-object-timestamp new))
  old)
