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

;;; Location designators are resolved a little bit differently than
;;; object designators (at least for now.) To resolve, the
;;; cram/reasoning prolog predicate desig-loc is used. All solutions
;;; are provided an can be accessed with next-solution. A mechanism is
;;; provided to post-process the solutions from reasoning, e.g. to
;;; sort according to eucledian distance.

(defclass location-designator (designator) ())
(register-designator-type location location-designator)

(defmethod equate ((desig-1 location-designator) (desig-2 location-designator))
  (or ;; (when (eq (desig-prop-value desig-1 'to) 'see)
      ;;   (< (jlo:euclidean-distance (reference desig-1) (reference desig-2))
      ;;      0.10))
      (call-next-method)))

(defmethod reference ((desig location-designator))
  (unless (slot-value desig 'data)
    (setf (slot-value desig 'data)
          (lazy-mapcar (alexandria:compose #'jlo:id
                                           (curry #'var-value '?loc))
                       (prolog `(desig-loc ,desig ?loc)))))
  (or (lazy-car (slot-value desig 'data))
      (error "Unable to resolve location designator `~a'" (description desig))))

(defmethod next-solution ((desig location-designator))
  (with-slots (data) desig
    (when (car (cut:lazy-cdr data))
      (cond ((children desig)
             (car (children desig)))
            (t
             (let ((new-desig (make-designator 'location (description desig) desig)))
               (setf (slot-value new-desig 'data) (cut:lazy-cdr data))
               new-desig))))))

(defmethod merge-designators ((desig-1 location-designator) (desig-2 location-designator))
  (let ((desig (make-designator 'location (description desig-1))))
    (setf (slot-value desig 'data)
          (lazy-append (slot-value desig-1 'data)
                       (slot-value desig-2 'data)))
    ;; Todo: link the designators in a parent <-> child relationship to fix equate
    desig))
