;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

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
