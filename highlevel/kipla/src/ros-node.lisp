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

(in-package :kipla)

(defvar *ros-init-functions* (make-hash-table :test 'cl:eq))
(defvar *ros-cleanup-functions* (make-hash-table :test 'cl:eq))


(defmacro register-ros-init-function (name)
  `(setf (gethash ',name *ros-init-functions*)
         (symbol-function ',name)))

(defmacro register-ros-cleanup-function (name)
  `(setf (gethash ',name *ros-cleanup-functions*)
         (symbol-function ',name)))

(defun startup-ros ()
  (start-ros-node "kipla")
  (loop for f being the hash-values of *ros-init-functions*
     do (funcall f)))

(defun shutdown-ros ()
  (loop for f being the hash-values of *ros-cleanup-functions*
     do (funcall f))
  (shutdown-ros-node))
