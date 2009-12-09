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

(defvar *object-designator-resolvers* nil)

(defstruct object-desig-resolver
  name namespace function)

(defclass object-designator (designator) ())
(register-designator-type object object-designator)

(defun add-obj-desig-resolver (resolver)
  (let ((prev-resolver (find (object-desig-resolver-name resolver)
                             *object-designator-resolvers*
                             :key #'object-desig-resolver-name)))
    (if prev-resolver
        (setf (object-desig-resolver-namespace prev-resolver)
              (object-desig-resolver-namespace resolver)
              (object-desig-resolver-function prev-resolver)
              (object-desig-resolver-function resolver))
        (push resolver *object-designator-resolvers*))))

(defmacro register-object-desig-resolver (name namespace (prev-param desig-param) &body body)
  "Registers a designator resolver. When converting the designator into something useful,
   i.e. into a format the perception routines can work with. It does
   this by reducing the designator to be resolved over the list of
   *object-designator-resolvers* the match the current designator
   namespace."
  `(add-obj-desig-resolver (make-object-desig-resolver
                            :name ',name :namespace ,namespace
                            :function (lambda (,prev-param ,desig-param)
                                        ,@body))))

(defun resolve-object-desig (desig namespace)
  (let ((info (reduce (lambda (prev resolver)
                        (when (eq (object-desig-resolver-namespace resolver)
                                  namespace)
                          (funcall (object-desig-resolver-function resolver)
                                   prev desig)))
                      *object-designator-resolvers*
                      :initial-value nil)))
    ;; (when (valid desig)
    ;;   (push (perceived-object-jlo (reference desig))
    ;;         (cop-desig-location-info-poses (cop-desig-info-location info))))
    info))

(defmethod reference ((desig object-designator))
  (or (slot-value desig 'data)
      (error "Designator does not reference an object.")))

;; (defmethod equate ((desig-1 object-designator) (desig-2 object-designator))
;;   (or (call-next-method)
;;       (desig-compatible-descriptions (description desig-1)
;;                                      (description desig-2))))

(defun desig-compatible-descriptions (desc-1 desc-2)
  (multiple-value-bind (desc-1 desc-2) (if (< (list-length desc-1)
                                              (list-length desc-2))
                                           (values desc-1 desc-2)
                                           (values desc-2 desc-1))
    (loop for (prop-1-name prop-1-val) in desc-1
       when (let ((prop-2-val (find prop-1-name desc-2 :key #'car)))
              (when prop-2-val
                (not (eql prop-1-val prop-2-val))))
       do (return nil)
       finally (return t))))
