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

(defgeneric update-belief (data &optional reference-object)
  (:documentation "Updates the belief state and returns the anchored
                   instance of data."))

(defvar *believed-occasions* (make-fluent :name 'believed-occasions))

(defun clear-belief ()
  (clear-object-belief)
  (setf (value *believed-occasions*) nil))

(defun assert-occasion (occ)
  (kipla:log-msg :info "Asserting occasion `~a'" occ)
  (pushnew occ (value *believed-occasions*)
           :test (rcurry #'pat-match-p :test #' equate)))

(defun retract-occasion (occ)
  (kipla:log-msg :info "Retracting occasion `~a'" occ)
  (setf (value *believed-occasions*)
        (remove occ (value *believed-occasions*)
                :test (rcurry #'pat-match-p :test #'equate))))

(defun holds (occ)
  (find occ (value *believed-occasions*) :test (rcurry #'pat-match-p :test #'equate)))
