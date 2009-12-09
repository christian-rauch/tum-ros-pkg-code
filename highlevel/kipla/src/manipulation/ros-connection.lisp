;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Piotr Esden-Tempski <esdentem@cs.tum.edu>
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

(def-actionlib-interface left-arm "/left_arm"
  'cogman_msgs-msg:<ARMHANDACTION>)

(register-ros-init-function left-arm-init)
(register-ros-cleanup-function left-arm-cleanup)

(def-actionlib-interface right-arm "/right_arm"
  'cogman_msgs-msg:<ARMHANDACTION>)

(register-ros-init-function right-arm-init)
(register-ros-cleanup-function right-arm-cleanup)

(defun execute-arm-action (action-description)
  (let* ((action-goal (make-instance 'cogman_msgs-msg:<armhandgoal>
                        :command (trajectory-type action-description)
                        :pose_name (format nil "~a_~a"
                                           (string-downcase (symbol-name (side action-description)))
                                           (stored-pose-type action-description))
                        ;; :joint_angles (unused)
                        :hand_primitive (hand-primitive action-description)
                        :object_type (object-type action-description)
                        :end_effector_loid (id (end-effector-pose action-description))
                        :obstacle_ids (let ((content (remove (jlo:id (end-effector-pose action-description))
                                                             (remove-duplicates
                                                              (mapcar (alexandria:compose #'jlo:id #'perceived-object-jlo) *perceived-objects*)))))
                                        (make-array (list-length content) :initial-contents content))
                        :distance (grasp-distance action-description)
                        ;; :supporting_plane (unused)
                        ))
         (result (ecase (side action-description)
                              (:left (log-msg :info "[manipulation action] sending left goal.")
                                     (left-arm-send-goal action-goal))
                              (:right (log-msg :info "[manipulation action] sending right goal.")
                                      (right-arm-send-goal action-goal)))))
    (when result
      (list (intern (string-upcase (cogman_msgs-msg:situation-val result))
                    (find-package :keyword))
            (map 'list #'identity (cogman_msgs-msg:better_base_ids-val result))
            (cogman_msgs-msg:distance_to_goal-val result)))))
