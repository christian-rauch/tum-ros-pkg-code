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

(define-condition destination-location-occupied (plan-error) ())

(defun optimized-manipulation-location (side &optional obj)
  (let* ((robot->obj (and obj
                          (jlo:frame-query (jlo:make-jlo :name "/base_link")
                                           (reference obj)))))
    (when (or (not obj)
              (= (aref (vision_msgs-msg:pose-val robot->obj) 7)
                 0)
              (> (abs (/ (aref (vision_msgs-msg:pose-val robot->obj) 3)
                         (aref (vision_msgs-msg:pose-val robot->obj) 7)))
                 (tan (ecase side
                        (:right (- (/ (* 30 pi)
                                      180)))
                        (:left (/ (* 30 pi)
                                  180))))))
      (let ((new-lo (jlo:make-jlo-rpy :parent (jlo:make-jlo :name "/base-link")
                                      :yaw (ecase side
                                             (:right (/ (* 30 pi)
                                                        180))
                                             (:left (- (/ (* 30 pi)
                                                          180)))))))
        (make-designator 'location `((jlo ,new-lo)))))
    ;; (cond (ignore-alternative-poses
    ;;        (setf pick-up-loc (next-solution pick-up-loc)))
    ;;       (t
    ;;        (setf ignore-alternative-poses t)
    ;;        (setf pick-up-loc
    ;;              (merge-designators (make-designator 'location `((jlo-list ,(alternative-poses f))))
    ;;                                 pick-up-loc))))
))

(defun location-at-side (loc)
  "Returns if a location is left or right of the robot."
  (let ((base->loc (jlo:frame-query (jlo:make-jlo :name "/base_link")
                                    (reference loc))))
    (if (>= (jlo:pose base->loc 1 3) 0)
        :left
        :right)))

(defun alternative-put-down-location (loc offset)
  (let ((new-loc (jlo:frame-query (jlo:make-jlo :name "/base_link")
                                  (reference loc))))
    (incf (jlo:pose new-loc 1 3) offset)
    (make-designator 'location `((jlo ,new-loc)))))

(defun clusters-within-range (loc range)
  (loop for c in *perceived-objects*
     when (< (jlo:euclidean-distance (reference loc)
                                     (perceived-object-jlo c))
             range)
     collecting c))

(def-goal (achieve (object-in-hand ?obj ?side))
  (log-msg :info "(achieve (object-in-hand))")
  (let ((retry-count 0)
        (alternative-poses-cnt 0))
    (with-failure-handling
        ((object-lost (f)
           (declare (ignore f))
           (log-msg :warn "Object lost.")
           (when (< (incf retry-count) 3)
             (retry)))
         (manipulation-failed (f)
           (log-msg :warn "Manipulation action failed. ~a" f)
           (setf alternative-poses-cnt 0)
           (say "Failed to grasp. I try again.")           
           (achieve `(arms-at ,(make-designator 'action `((type trajectory) (pose open) (side ,?side)))))
           (achieve `(arm-parked ,?side))
           (when (< (incf retry-count) 3)
             (retry))))
      (setf ?obj (perceive ?obj))
      (with-designators ((pick-up-loc (location `((to reach) (obj ,?obj))))
                         (open-trajectory (action `((type trajectory) (pose open) (side ,?side))))
                         (grasp-trajectory (action `((type trajectory) (to grasp) (obj ,?obj) (side ,?side))))
                         (lift-trajectory (action `((type trajectory) (to lift) (obj ,?obj) (side ,?side)))))
        (with-failure-handling
            ((manipulation-pose-unreachable (f)
               (declare (ignore f))
               (say "Cannot reach object. Trying another position.")
               (log-msg :warn "Got unreachable grasp pose. Trying alternatives")
               (let ((optimized-loc (optimized-manipulation-location ?side ?obj)))
                 (when (and (< alternative-poses-cnt 3)
                            optimized-loc (reference optimized-loc))
                   (incf alternative-poses-cnt)
                   (setf pick-up-loc optimized-loc)
                   ;; ;; Evil: assert already in parked position. __EVIL__
                   ;; (assert-occasion `(arm-parked :both))
                   (retry)))))
          (at-location (pick-up-loc)
            (log-msg :info "object-in-hand: trying to grasp object.")
            (achieve `(arms-at ,open-trajectory))
            (achieve `(arms-at ,grasp-trajectory))
            (achieve `(arms-at ,lift-trajectory))))))
    (retract-occasion `(object-placed-at ,?obj))
    (assert-occasion `(object-in-hand ,?obj ,?side)))
  ?obj)

(def-goal (achieve (object-placed-at ?obj ?loc))
  (log-msg :info "(achieve (object-placed-at))")
  (setf ?obj (car (youngest-children ?obj)))
  (let ((object-in-hand-occasion (holds `(object-in-hand ,?obj ?_)))
        (alternative-poses-cnt 0)
        (retry-count 0))
    (assert object-in-hand-occasion ()
            "The object `~a ~a' needs to be in the hand before being able to place it."
            ?obj (description ?obj))
    (destructuring-bind (sym obj side) object-in-hand-occasion
      (declare (ignore sym))
      (with-failure-handling
          ((manipulation-failed (f)
             (declare (ignore f))
             (log-msg :warn "Manipulation action failed.")
             (say "Failed to put down. I try again.")
             (with-designators ((open-trajectory (action `((type trajectory) (pose open) (side ,side)))))
               (achieve `(arms-at ,open-trajectory)))
             (when (< (incf retry-count) 3)
               (retry))))
        (with-designators ((put-down-loc (location `((to reach) (location ,?loc))))
                           (open-trajectory (action `((type trajectory) (pose open) (side ,side))))
                           (put-down-trajectory (action `((type trajectory) (to put-down)
                                                          (obj ,obj) (at ,?loc) (side ,side))))
                           (hand-open-trajectory (action `((type trajectory) (to open) (gripper ,side))))
                           (unhand-trajectory (action `((type trajectory) (to lift) (side ,side))))
                           ;; (clusters (object `((type cluster) (matches 10) (at ,?loc))))
                           )
          (with-failure-handling
              ((manipulation-pose-unreachable (f)
                 (declare (ignore f))
                 (say "Cannot reach position. Trying to turn for better position.")
                 (log-msg :warn "Got unreachable put-down pose. Trying alternatives")
                 (let ((optimized-loc (optimized-manipulation-location side)))
                   (when (and (< alternative-poses-cnt 3)
                              optimized-loc (reference optimized-loc))
                     (log-msg :info "Setting alternative pose to ~a" (reference optimized-loc))
                     (setf put-down-loc optimized-loc)
                     (retry))))
               ;; (destination-location-occupied (e)
               ;;   (declare (ignore e))
               ;;   (log-msg :warn "Destination location occupied.")
               ;;   (setf ?loc (alternative-put-down-location
               ;;               ?loc (if (eq side :left)
               ;;                        0.10
               ;;                        -0.10))))
               )
            (at-location (put-down-loc)
              (achieve `(looking-at ,(reference ?loc)))
              ;; (perceive clusters)
              ;; (when (clusters-within-range 0.1)
              ;;   (fail (make-condition 'destination-location-occupied)))
              (achieve `(arms-at ,open-trajectory))
              (achieve `(arms-at ,put-down-trajectory))
              (achieve `(arms-at ,hand-open-trajectory))
              (achieve `(arms-at ,unhand-trajectory))))
          (retract-occasion `(object-in-hand ,obj ?_))
          (assert-occasion `(object-placed-at ,obj ,?loc)))))))

(def-goal (achieve (arm-parked ?side))
  (flet ((park-both-arms ()
           (with-designators ((open (action `((type trajectory) (pose open) (side :both))))
                              (parking (action `((type trajectory) (pose parked) (side :both))))
                              (hand-open (action `((type trajectory) (to open) (gripper :both))))
                              (hand-closed (action `((type trajectory) (to close) (gripper :both)))))
             (achieve `(arms-at ,hand-open))
             (achieve `(arms-at ,open))
             (achieve `(arms-at ,hand-closed))
             (achieve `(arms-at ,parking))
             (assert-occasion `(arm-parked :left))
             (assert-occasion `(arm-parked :right))))
         (park-one-arm (side)
           (unless (holds `(arm-parked ,side))
             (if (holds `(object-in-hand ?_ ,side))
                 (with-designators ((parking (action `((type trajectory) (to carry) (side ,side)))))
                   (log-msg :info "going to left carry pose")
                   (achieve `(arms-at ,parking)))
                 (with-designators ((hand-open (action `((type trajectory) (to open) (gripper ,side))))
                                    (hand-closed (action `((type trajectory) (to close) (gripper ,side))))
                                    (open (action `((type trajectory) (pose open) (side ,side))))
                                    (parking (action `((type trajectory) (pose parked) (side ,side)))))
                   (achieve `(arms-at ,hand-open))
                   (achieve `(arms-at ,open))
                   (achieve `(arms-at ,hand-closed))
                   (achieve `(arms-at ,parking))))
             (assert-occasion `(arm-parked ,side)))))
    (log-msg :info "(achieve (arm-parked ~a))" ?side)
    (let ((parked-arms (remove nil (list (holds `(arm-parked :left))
                                         (holds `(arm-parked :right)))))
          (carried-objs (holds `(object-in-hand ?_ ?_))))
      (log-msg :info "Already parked arms ~a~%" parked-arms)
      (case ?side
        (:both (cond ((or parked-arms carried-objs)
                      (park-one-arm :left)
                      (park-one-arm :right))
                     (t
                      (park-both-arms))))
        ((:left :right) (park-one-arm ?side))))))

(def-goal (achieve (arms-at ?traj))
  (let ((side (desig-prop-value ?traj 'side)))
    (retract-occasion `(arms-at ?_))    
    (pm-execute 'manipulation ?traj)
    (when (member side '(:both :left))
      (retract-occasion `(arm-parked :left)))
    (when (member side '(:both :right))
      (retract-occasion `(arm-parked :right)))
    (retract-occasion `(arms-at ?_))
    (assert-occasion `(arms-at ,?traj))))
