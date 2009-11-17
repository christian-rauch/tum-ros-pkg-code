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

(defun distance-to-drive (goal)
  (let* ((loc-1 (reference goal))
         (current-loc-occasion (holds `(loc Robot ?_)))
         (loc-2 (and current-loc-occasion (reference (third current-loc-occasion)))))
    (format t "~%distance-to-drive ~a~%" (and loc-1 loc-2 (jlo:euclidean-distance loc-1 loc-2)))
    (cond ((and loc-1 loc-2)
           (jlo:euclidean-distance loc-1 loc-2))
          (t
           100))))

(def-goal (achieve (loc Robot ?loc))
  (log-msg :info "Driving to location.")
  (unless (reference ?loc)
    (fail "Location designator invalid."))
  (when (> (distance-to-drive ?loc) 0.5)
    (log-msg :info "Distance to drive: ~a, parking arms.~%"
             (distance-to-drive ?loc))
    (achieve `(looking-at :forward))
    (achieve '(arm-parked :both)))
  (pm-execute 'navigation ?loc)
  (retract-occasion `(loc Robot ?_))
  (assert-occasion `(loc Robot ,?loc)))

(def-goal (achieve (loc ?obj ?loc))
  (log-msg :info "(achieve (loc ?obj ?loc)")
  (log-msg :info "?obj `~a' ?loc `~a'" (description ?obj) (description ?loc))
  (let ((retry-count 0))
    (with-failure-handling
        ((object-lost (f)
           (declare (ignore f))
           (when (< (incf retry-count) 3)
             (retry))))
      (achieve `(object-in-hand ,?obj :right))
      (achieve `(object-placed-at ,?obj ,?loc))))
  (retract-occasion `(loc ,?obj ?_))
  (assert-occasion `(loc ,?obj ,?loc)))
