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

(def-actionlib-interface navigation "/nav_pcontroller/nav_action"
  'nav_pcontroller-msg:<NAV_ACTIONACTION>)

(register-ros-init-function navigation-init)
(register-ros-cleanup-function navigation-cleanup)

(defvar *navigation-speed-fluent*
  (make-fluent :name 'navigation-speed-fluent :value 0))
(defvar *navigation-distance-to-goal-fluent*
  (make-fluent :name 'navigation-distance-to-goal-fluent :value 0))

(defun navigation-execute-goal (lo_id)
  (log-msg :info "executing nav action")
  (navigation-send-goal 
   (make-instance 'nav_pcontroller-msg:<nav_actiongoal>
     :target_lo (make-instance 'std_msgs-msg:<uint64>
                  :data lo_id))
   (lambda (x)
     (setf (value *navigation-speed-fluent*)
           (std_msgs-msg:data-val
            (nav_pcontroller-msg:speed-val
             (nav_pcontroller-msg:feedback-val x))))
     (setf (value *navigation-distance-to-goal-fluent*)
           (std_msgs-msg:data-val
            (nav_pcontroller-msg:distance-val
             (nav_pcontroller-msg:feedback-val x)))))))
