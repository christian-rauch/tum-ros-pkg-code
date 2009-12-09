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

(defclass trajectory-action ()
  ((side :initform nil :reader side :initarg :side
         :documentation "Which arm :right or :left")
   (trajectory-type :initform "" :reader trajectory-type :initarg :trajectory-type
                    :documentation "The type of the action: :cartesian, :reach-primitive and :joint-pos")
   (stored-pose-type :initform "" :reader stored-pose-type :initarg :store-pose-type
                     :documentation "One of the stored joint based poses, :navigate...")
   (object-type :initform "" :reader object-type :initarg :object-type
                :documentation "The object type (e.g :top)")
   (hand-primitive :initform "" :reader hand-primitive :initarg :hand-primitive
                   :documentation "The hand primitive to use.")
   (end-effector-pose :initform 0 :reader end-effector-pose :initarg :end-effector-pose
                      :documentation "JLO of the commanded end effector pose.")
   (obstacles :initform nil :reader obstacles :initarg :obstacles
              :documentation "List of jlo objects that describe obstacles")
   (grasp-distance :initform 0 :reader grasp-distance :initarg :grasp-distance
                   :documentation "Stop at this distance to the commanded pose.")
   (supporting-plane :initform 0 :reader supporting-plane :initarg :supporting-plane
                     :documentation "Height of the supporting plane (table).")))

(defun copy-trajectory-action (ref)
  (make-instance 'trajectory-action
    :side  (slot-value ref 'side)
    :trajectory-type (slot-value ref 'trajectory-type)
    :store-pose-type (slot-value ref 'stored-pose-type)
    :object-type (slot-value ref 'object-type)
    :hand-primitive (slot-value ref 'hand-primitive)
    :end-effector-pose (slot-value ref 'end-effector-pose)
    :obstacles (slot-value ref 'obstacles)
    :grasp-distance (slot-value ref 'grasp-distance)
    :supporting-plane (slot-value ref 'supporting-plane)))

(defun calculate-put-down-end-effector-lo (supporting obj)
  (assert (typep supporting 'location-designator))
  (assert (typep obj 'object-designator))
  (with-desig-props (at) obj
    (let ((z-dist (jlo:component-distance (perceived-object-jlo (reference obj)) (reference at) :axis :z))
          (base->supporting (jlo:frame-query (jlo:make-jlo :name "/base_link") (reference supporting))))
      (incf (jlo:pose base->supporting 2 3) (+ z-dist 0.01)))))

;;; Possible trajectory configs:
;;; (to grasp)
;;; (to unhand)
;;; (to carry)
;;; (to navigate)

(def-fact-group manipulation-designators
  (<- (manip-desig? ?desig)
    (lisp-pred typep ?desig action-designator)
    (desig-prop ?desig (type trajectory)))

  ;; (grasp-info ?obj-desig ?object-type ?hand_primitive ?distance-to-grasp)
  (<- (grasp-info ?obj "Mug" "3pinch" 0.14)
    (desig-prop ?obj (type mug)))

  (<- (grasp-info ?obj "Jug" "3pinch" 0.05)
    (desig-prop ?obj (type jug)))

  (<- (grasp-info ?obj "IceTea" "3pinch" 0.10)
    (desig-prop ?obj (type icetea)))

  (<- (grasp-info ?obj "front" "3pinch" 0.04)
    (desig-prop ?obj (type coke)))
  
  (<- (action-desig ?desig ?act)
    (manip-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (side ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "reach_primitive")
    (obj-desig-location ?obj ?obj-loc)
    (slot-value ?act end-effector-pose ?obj-loc)
    (grasp-info ?obj ?object-type ?hand-primitive ?dist)
    (slot-value ?act object-type ?object-type)
    (slot-value ?act hand-primitive ?hand-primitive)
    (slot-value ?act grasp-distance ?dist)
    ;; Todo: infer supporting plane height (with liswip)
    )

  (<- (action-desig ?desig ?act)
    (manip-desig? ?desig)
    (or (desig-prop ?desig (to navigate))
        (desig-prop ?desig (pose parked)))
    (desig-prop ?desig (side ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "arm_joint_pose")
    (slot-value ?act stored-pose-type "parking"))

  (<- (action-desig ?desig ?act)
    (manip-desig? ?desig)
    (desig-prop ?desig (pose open))
    (desig-prop ?desig (side ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "arm_joint_pose")
    (slot-value ?act stored-pose-type "open"))

  (<- (action-desig ?desig ?act)
    (manip-desig? ?desig)
    (desig-prop ?desig (to lift))
    (desig-prop ?desig (side ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "lift")
    (slot-value ?act grasp-distance 0.20))

  (<- (action-desig ?desig ?act)
    (manip-desig? ?desig)
    (desig-prop ?desig (to carry))
    (desig-prop ?desig (side ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "arm_cart_pose")
    (slot-value ?act stored-pose-type "carry"))

  (<- (action-desig ?desig ?act)
    (manip-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (side ?side))
    (desig-prop ?desig (at ?loc-desig))
    (desig-prop ?desig (obj ?obj-desig))
    (grasp-info ?obj-desig ?obj-type ?_ ?_)
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "put_down")
    (slot-value ?act object-type ?obj-type)
    (lisp-fun calculate-put-down-end-effector-lo
              ?loc-desig ?obj-desig ?jlo)
    (slot-value ?act end-effector-pose ?jlo))

  (<- (action-desig ?desig ?act)
    (manip-desig? ?desig)
    (desig-prop ?desig (to open))
    (desig-prop ?desig (gripper ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "hand_primitive")
    (slot-value ?act hand-primitive "open_thumb90"))

  (<- (action-desig ?desig ?act)
    (manip-desig? ?desig)
    (desig-prop ?desig (to close))
    (desig-prop ?desig (gripper ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "hand_primitive")
    (slot-value ?act hand-primitive "3pinch")))
