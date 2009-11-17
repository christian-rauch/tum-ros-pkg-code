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

(declare-goal perceive (obj)
  (declare (ignore obj)))

(def-goal (perceive ?obj-desig)
  (log-msg :info "Perceiving `~a'" (description ?obj-desig))
  (with-designators ((loc (location `((to see) (obj ,?obj-desig)))))
    (with-failure-handling
        ((object-not-found (e)
           (declare (ignore e))
           (log-msg :warn "Object not found failure.")
           (setf loc (next-solution loc))
           (when (and loc (reference loc))
             (log-msg :info "Retrying at different location.")
             (retry))
           (log-msg :warn "Failing at object-not-found failure.")))
      (at-location (loc)
        (let ((new-desig (pm-execute 'perception ?obj-desig)))
          (assert-occasion `(perceived ,new-desig))
          new-desig)))))
