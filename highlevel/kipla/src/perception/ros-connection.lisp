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

(defvar *cop-output-queue* (make-fluent :name '*cop-output-queue* :value nil))
(defvar *cop-parameters* (make-hash-table :test 'cl:eq))

(defun cop-ros-init ()
  (subscribe "/kipla/cop_reply" 'vision_msgs-msg:<cop_answer> #'cop-reply-handler)
  (advertise "/head_controller/lookat_lo" 'std_msgs-msg:<uint64>)
  (setf (gethash :object_threshold *cop-parameters*)
        (get-param "/kipla/object_threshold" 0.4))
  (setf (gethash :max-object-distance *cop-parameters*)
        (get-param "/kipla/object_max_distance" 1.8)))

(register-ros-init-function cop-ros-init)

(defun cop-reply-handler (cop-reply)
  (push cop-reply (value *cop-output-queue*)))

(defun cop-query (query-info)
  (call-service "/tracking/in" 'vision_srvs-srv:cop_call
                :outputtopic "/kipla/cop_reply"
                :object_classes (make-array (list-length (cop-desig-query-info-object-classes query-info))
                                            :initial-contents (cop-desig-query-info-object-classes query-info))
                :object_ids (make-array (list-length (cop-desig-query-info-object-ids query-info))
                                        :initial-contents (cop-desig-query-info-object-ids query-info))
                :action_type 0
                :number_of_objects (cop-desig-query-info-matches query-info)
                :list_of_poses (make-array (list-length (cop-desig-query-info-poses query-info))
                                           :initial-contents (mapcar (lambda (id)
                                                                       (make-instance 'vision_msgs-msg:<apriori_position>
                                                                         :probability 0.9
                                                                         :positionid id))
                                                                     (cop-desig-query-info-poses query-info)))))

(defun cop-reply->object (reply)
  (make-perceived-object :jlo (jlo:make-jlo :id reply)
                         :properties (map 'list #'identity (vision_msgs-msg:classes-val reply))
                         :probability (vision_msgs-msg:probability-val reply)))

;; (defun look-at (pose-id)
;;   (publish "/head_controller/lookat_lo" (make-instance 'std_msgs-msg:<uint64> :data pose-id)))

(def-actionlib-interface ptu "ptu"
  'cogman_msgs-msg:<ptuaction>)

(register-ros-init-function ptu-init)
(register-ros-cleanup-function ptu-cleanup)

(defun look-at (lo_id)
  (ptu-send-goal (make-instance 'cogman_msgs-msg:<ptugoal>
                   :lo_id lo_id)))
