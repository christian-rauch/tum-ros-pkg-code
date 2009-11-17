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

(define-condition object-not-found (plan-error)
  ((object-desig :initarg :object-desig :initform nil :reader object-not-found-desig)))

(defun cop-search-for-object (query-info)
  (labels (;; (check-object-in-range (ref-lo lo range)
           ;;   (format t "check-obj-in-range~%")
           ;;   (< (jlo:euclidean-distance ref-lo lo) range))
           (get-anchored-clusters ()
             (setf (value *cop-output-queue*) nil)
             (with-designators ((clusters (object '((type cluster) (matches 10)))))
               (let ((query-info (cop-desig-info-query (resolve-object-desig clusters :cop))))
                 (setf (cop-desig-query-info-poses query-info)
                       (list (jlo:id "/RightEyeCalc")))
                 (cop-query query-info))
               (wait-for *cop-output-queue*)
               (let ((cop-reply (pop (value *cop-output-queue*))))
                 (assert (equal (vision_msgs-msg:error-val cop-reply) ""))
                 (mapcar (alexandria:compose #'update-belief #'cop-reply->object)
                         (map 'list #'identity (vision_msgs-msg:found_poses-val cop-reply)))
                 ;; (mapcar (alexandria:compose #'update-belief #'cop-reply->object)
                 ;;         (remove-if-not
                 ;;          (lambda (cop-reply)
                 ;;            (format t "cop-reply: ~a~%" cop-reply)
                 ;;            (check-object-in-range "/RightEyeCalc" (vision_msgs-msg:position-val cop-reply)
                 ;;                                   (gethash :max-object-distance *cop-parameters*)))
                 ;;          (map 'list #'identity (vision_msgs-msg:found_poses-val cop-reply))))
                 )))
           (get-anchored-objs (query-info &optional cluster)
             (setf (value *cop-output-queue*) nil)
             (when cluster
               (setf (cop-desig-query-info-poses query-info)
                     (list (perceived-object-lo-id cluster))))
             (cop-query query-info)
             (wait-for *cop-output-queue* :handle-missed-pulses :never)
             (let ((cop-reply (pop (value *cop-output-queue*))))
               (when (and cop-reply (equal (vision_msgs-msg:error-val cop-reply) ""))
                 (mapcar (alexandria:compose (rcurry #'update-belief cluster) #'cop-reply->object)
                         (map 'list #'identity (vision_msgs-msg:found_poses-val cop-reply))
                         ;; (remove-if-not (lambda (lo)
                         ;;                  (check-object-in-range "/RightEyeCalc" lo
                         ;;                                         (gethash :max-object-distance *cop-parameters*)))
                         ;;                (map 'list #'identity (vision_msgs-msg:found_poses-val cop-reply)))
                         )))))
    (handler-case
        (cond ((cop-desig-query-info-poses query-info)
               (car (sort (get-anchored-objs query-info query-info) 
                          #'> :key #'perceived-object-probability)))
              (t
               (car (sort (loop for cluster in (get-anchored-clusters)
                             appending (get-anchored-objs query-info cluster)) 
                          #'> :key #'perceived-object-probability))))
      (error (e)
        (declare (ignore e))
        nil))))

(defun cop-result->designator (desig obj)
  (let ((new-desig (make-designator 'object (description desig) desig)))
    ;; Todo: Merge the object properties with the desinator's props
    ;; Todo: Use weak references here to make desigs gc-able
    (cond ((null (perceived-object-desig obj))
           (setf (perceived-object-desig obj) new-desig))
          ((not (equate desig (perceived-object-desig obj)))
           ;; Designators seem to reference the same object but cannot
           ;; be equated yet. Allign them so that they can be equated
           ;; in the future.
           (push (original-desig desig) (children (perceived-object-desig obj)))))
    (setf (slot-value new-desig 'data)
          obj)
    (setf (slot-value new-desig 'valid) t)
    new-desig))

(def-process-module perception (pm)
  ;; This process module receives an instance of cop-desig-info and
  ;; returns a designator. It updates the robot's belief state with
  ;; all information gathered during execution.
  (let ((input (value (slot-value pm 'input))))
    (assert (typep input 'object-designator))
    (let* ((cop-desig-info (resolve-object-desig input :cop))
           (ptu-poses (cop-desig-location-info-poses (cop-desig-info-location cop-desig-info)))
           (query-info (cop-desig-info-query cop-desig-info)))
      (log-msg :info "[Perception process module] looking at poses: ~a~%" ptu-poses)
      (assert ptu-poses)
      (clear-object-belief)
      (when (member :perception *kipla-features*)
        (loop for ptu-pose in ptu-poses
           do (progn
                (look-at ptu-pose)
                (sleep 2)
                (let ((result (cop-search-for-object query-info)))
                  (when result
                    (log-msg :info "[Perception process module] found an object.")
                    (return (cop-result->designator (cop-desig-info-designator cop-desig-info)
                                                    result)))))
           finally (fail (make-condition 'object-not-found :object-desig input)))))))
