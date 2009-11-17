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

(defclass action-designator (designator) ())
(register-designator-type action action-designator)

(defmethod reference ((desig action-designator))
  (or (slot-value desig 'data)
      (let ((action-desig (var-value '?act
                                     (lazy-car (prolog `(action-desig ,desig ?act))))))
        (when (is-var action-desig)
          (error "Cannot resolve action designator."))
        (setf (slot-value desig 'data) action-desig))))

(defmethod equate ((desig-1 action-designator) (desig-2 action-designator))
  (equal (description desig-1) (description desig-2)))
