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

(def-fact-group designators
  (<- (desig-prop ?desig (?prop-name ?prop))
    (is-bound ?desig)
    (is-bound ?prop-name)
    (lisp-fun desig-prop-value ?desig ?prop-name ?prop)
    (lisp-pred identity ?prop)))

(defun prolog-get-slot-value (obj slot)
  (slot-value obj slot))

(defun prolog-set-slot-value (obj slot new-value)
  (setf (slot-value obj slot) new-value))

(def-fact-group lisp-data-structs

  (<- (slot-value ?obj ?slot ?value)
    (is-bound ?obj)
    (is-bound ?slot)
    (not (is-bound ?value))
    (lisp-fun prolog-get-slot-value ?obj ?slot ?value))
  (<- (slot-value ?obj ?slot ?value)
    (is-bound ?obj)
    (is-bound ?slot)
    (is-bound ?value)
    (lisp-fun prolog-set-slot-value ?obj ?slot ?value ?_))

  (<- (instance-of ?type ?obj)
    (is-bound ?type)
    (not (is-bound ?obj))
    (lisp-fun make-instance ?type ?obj))
  (<- (instance-of ?type ?obj)
    (not (is-bound ?type))
    (is-bound ?obj)
    (lisp-pred type-of ?obj ?type))
  (<- (instance-of ?type ?obj)
    (is-bound ?type)
    (is-bound ?obj)
    (lisp-pred typep ?obj ?type)))
