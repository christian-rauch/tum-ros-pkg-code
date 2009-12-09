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

(define-condition navigation-failure (plan-error)
  ((location :initarg :location :initform nil :reader navigation-failure-location)))

(define-condition location-not-reached-failure (navigation-failure) ())

(define-condition location-reached-but-not-terminated (plan-error) ())

(defun get-nav-waypoints (goal)
  (let ((base-link (jlo:make-jlo :name "/base_link"))
        (map (jlo:make-jlo :name "/map")))
    (append (when (< (jlo:euclidean-distance base-link map)
                     (jlo:euclidean-distance base-link goal))
              (list map))
            (list goal))))

(defun approach-waypoint (goal last? &optional (threshold 0.3))
  (setf (value *navigation-distance-to-goal-fluent*) 100)
  (if last?
      (navigation-execute-goal goal)
      (pursue
        (navigation-execute-goal goal)
        (wait-for (< *navigation-distance-to-goal-fluent* threshold)))))

(def-process-module navigation (pm)
  ;; This process module navigates the robot to the location given as
  ;; input.
  (with-slots (input) pm
    (log-msg :info "[Navigation process module] received input ~a~%" (reference (value input)))
    (when (member :navigation *kipla-features*)
      (with-failure-handling
          ((location-reached-but-not-terminated (c)
             (declare (ignore c))
             (return)))
        (let ((waypoints (get-nav-waypoints (reference (value input)))))
          (log-msg :info "[Navigation process module] waypoints `~a'" waypoints)
          (pursue
            (loop while waypoints
               do
                 (log-msg :info "Drive to waypoint `~a'" (car waypoints))
                 (approach-waypoint (car waypoints) (not (cdr waypoints)))
                 (sleep 0.1)
                 (setf waypoints (cdr waypoints)))
            (seq
              (whenever ((< *navigation-speed-fluent* 0.0001))
                (log-msg :info "[Navigation process module] speed dropped below threshold. Failing in 5s~%")
                ;; (when (< (value *navigation-distance-to-goal-fluent*) 0.15)
                ;;   (fail (make-condition 'location-reached-but-not-terminated)))
                (sleep 5)
                (when (< (value *navigation-speed-fluent*) 0.0001)
                  (fail (make-condition 'location-reached-but-not-terminated)))
                (log-msg :info "[Navigation process module] recovering from low speed.")))))))
    (log-msg :info "[Navigation process module] returning.~%")))
