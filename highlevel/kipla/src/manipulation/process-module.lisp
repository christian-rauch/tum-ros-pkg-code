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

(define-condition object-lost (simple-plan-error) ())
(define-condition manipulation-failed (simple-plan-error) ())
(define-condition manipulation-pose-unreachable (simple-plan-error)
  ((alternative-poses :initform nil :initarg :alternative-poses
                       :reader alternative-poses)))

(defvar *manipulation-action-designator* nil)

(def-process-module manipulation (pm)
  ;; This process module navigates the robot to the location given as
  ;; input.
  (flet ((check-result (action-result)
           (destructuring-bind (action-result better-lo-ids distance-to-goal)
               action-result
             (declare (ignore distance-to-goal))
             (case action-result
               (:could-not-reach (fail (make-condition 'manipulation-pose-unreachable
                                                       :format-control "Manipulation pose unreachable."
                                                       :alternative-poses better-lo-ids)))
               (:could-not-grasp (fail (make-condition 'manipulation-failed
                                                       :format-control "Grasp failed.")))
               (:cancelled (fail (make-condition 'manipulation-failed
                                                 :format-control "Manipulation was canceled.")))
               (t (log-msg :info "Action `~a', result: ~a"
                           (description (value (slot-value pm 'input)))
                           action-result))))))
    ;; We need to fix this. Why can input become nil?
    (when (value (slot-value pm 'input))
      (let ((action (reference (value (slot-value pm 'input)))))
        (log-msg :info "[Manipulation process module] received input ~a~%"
                 (description (value (slot-value pm 'input))))
        (when (member :manipulation *kipla-features*)
          (setf *manipulation-action-designator* (value (slot-value pm 'input)))
          (with-failure-handling
              ((goal-is-lost-err (e)
                 (declare (ignore e))
                 (log-msg :warn "[Manipulation process module] received goal-is-lost-err condition.")
                 (fail (make-condition 'manipulation-failed :format-control "manipulation failed."))))
            (ecase (side action)
              ((:left :right) (check-result (execute-arm-action action)))
              (:both (let ((left (copy-trajectory-action action))
                           (right (copy-trajectory-action action)))
                       (setf (slot-value left 'side) :left)
                       (setf (slot-value right 'side) :right)
                       (par
                         (check-result (execute-arm-action left))
                         (check-result (execute-arm-action right)))))))))))
  (log-msg :info "[Manipulation process module] returning."))
