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

(in-package :jlo)

(defvar *requested-jlo-objects* nil)

(defclass jlo ()
  ((id :initarg :id :initform 0 :reader id)
   (name :initarg :name :initform nil :reader name)
   (partial-lo :initform nil :reader partial-lo)))

(defmethod id :before ((jlo jlo))
  (unless (slot-value jlo 'id)
    (setf (slot-value jlo 'id)
          (vision_msgs-msg:id-val (partial-lo jlo)))))

(defmethod name :before ((jlo jlo))
  (unless (slot-value jlo 'name)
    (setf (slot-value jlo 'name)
          (vision_msgs-msg:name-val (partial-lo jlo)))))

(defmethod partial-lo :before ((jlo jlo))
  (unless (slot-value jlo 'partial-lo)
    (unless (or (slot-value jlo 'id)
                (slot-value jlo 'name))
      (error "jlo underspecified. Please specify either `id' or `name' slot."))
    (setf (slot-value jlo 'partial-lo)
          (query (or (slot-value jlo 'id)
                     (slot-value jlo 'name))))
    (jlo-register-gc jlo)))

(defun jlo-register-gc (jlo)
  (let ((id (id jlo)))
    (assert (not (eql 0 id)) () "Cannot garbage collect jlo ids with id 0")
    (push (cons id (trivial-garbage:make-weak-pointer jlo))
          *requested-jlo-objects*)
    jlo))

(defun jlo-gc ()
  nil
  ;; (let ((removed-ids
  ;;        (loop for (id . val) in *requested-jlo-objects*
  ;;           unless (trivial-garbage:weak-pointer-value val)
  ;;           collecting (prog1 id
  ;;                        (log-msg :info "Garbage collecting lo `~a'~%" id)
  ;;                        (unless (eql id 1)
  ;;                          ;; We are not allowed to del the world, so
  ;;                          ;; just avoid the call to save some
  ;;                          ;; resources.
  ;;                          (handler-case (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "del"
  ;;                                                      :query (make-instance 'vision_msgs-msg:<partial_lo>
  ;;                                                               :id id))
  ;;                            (error (e)
  ;;                              (log-msg :warn "Garbage collection of lo `~a' failed:~%~a~%" id e))))))))
  ;;   (setf *requested-jlo-objects* (delete-if (rcurry #'member removed-ids) *requested-jlo-objects* :key #'car)))
  )

(defun identity-matrix (parent-id &key (name "") (type 0))
  (create-matrix parent-id :name name :type type))

(defun shifted-identity-matrix (parent-id &key (name "") (type 0) (x 0) (y 0) (z 0))
  (create-matrix parent-id :type type :name name :x x :y y :z z))

(defun create-matrix (parent-id &key (name "") (type 0)
                          (x 0) (y 0) (z 0)
                          (roll 0) (pitch 0) (yaw 0))
  (let* ((cos-roll (cos roll))
         (sin-roll (sin roll))
         (cos-pitch (cos pitch))
         (sin-pitch (sin pitch))
         (cos-yaw (cos yaw))
         (sin-yaw (sin yaw))
         (lo (make-instance 'vision_msgs-msg:<partial_lo>
               :parent_id (id parent-id)
               :type type
               :name name
               :pose (make-array 16 :initial-contents
                                 `(,(* cos-yaw cos-pitch)
                                    ,(- (* cos-yaw sin-pitch sin-roll) (* sin-yaw cos-roll))
                                    ,(+ (* cos-yaw sin-pitch cos-roll) (* sin-yaw sin-roll))
                                    ,x

                                    ,(* sin-yaw cos-pitch)
                                    ,(+ (* sin-yaw sin-pitch sin-roll) (* cos-yaw cos-roll))
                                    ,(- (* sin-yaw sin-pitch cos-roll) (* cos-yaw sin-roll))
                                    ,y

                                    ,(- sin-pitch)
                                    ,(* cos-pitch sin-roll)
                                    ,(* cos-pitch cos-roll)
                                    ,z
                                    0 0 0 1))
               :cov (make-array 36 :initial-element 0))))
    (update lo)))

(defun print-pose (lo)
  (loop for x across (vision_msgs-msg:pose-val lo)
       for i from 1
     do (format t "~6,3F " x)
     when (eql (mod i 4) 0) do (format t "~%")))

(defun update (lo)
  (jlo-gc)  
  (jlo-register-gc
   (vision_srvs-srv:answer-val
    (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "update" :query lo))))

(defun query (id)
  (prog1
      (let ((result
             (typecase id
               (string (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "namequery"
                                     :query (make-instance 'vision_msgs-msg:<partial_lo> :name id)))
               (t (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "idquery"
                                :query (make-instance 'vision_msgs-msg:<partial_lo> :id id))))))
        (unless (equal (vision_srvs-srv:error-val result) "")
          (error "jlo call failed. ~a" (vision_srvs-srv:error-val result)))
        (vision_srvs-srv:answer-val result))
    (jlo-gc)))

(defun id (id)
  (typecase id
    (number id)
    (string (vision_msgs-msg:id-val (query id)))))

(defun frame-query (parent-id id)
  (prog1
      (let ((parent-id-num (id parent-id))
            (id-num (id id)))
        (cond ((eql id-num parent-id-num)
               (identity parent-id-num))
              (t
               (let ((result (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "framequery"
                                           :query (make-instance 'vision_msgs-msg:<partial_lo>
                                                    :id (id id) :parent_id (id parent-id)))))
                 (unless (equal (vision_srvs-srv:error-val result) "")
                   (error "jlo call failed. ~a" (vision_srvs-srv:error-val result)))
                 (jlo-register-gc
                  (vision_srvs-srv:answer-val result))))))
    (jlo-gc)))

(defun inlier? (reference-id obj-id &optional (threshold 0.05))
  (let ((ref-lo (query reference-id))
        (obj-lo (frame-query reference-id obj-id)))
    (and (< (abs (- (aref (vision_msgs-msg:cov-val ref-lo) 0)
                    (aref (vision_msgs-msg:pose-val obj-lo) 3)))
            threshold)
         (< (abs (- (aref (vision_msgs-msg:cov-val ref-lo) 7)
                    (aref (vision_msgs-msg:pose-val obj-lo) 7)))
            threshold)
         (< (abs (- (aref (vision_msgs-msg:cov-val ref-lo) 14)
                    (aref (vision_msgs-msg:pose-val obj-lo) 11)))
            threshold))))

(defun euclidean-distance (lo-1 lo-2)
  (let ((distance-lo (frame-query lo-1 lo-2)))
    (sqrt (+ (expt (aref (vision_msgs-msg:pose-val distance-lo) 3) 2)
             (expt (aref (vision_msgs-msg:pose-val distance-lo) 7) 2)
             (expt (aref (vision_msgs-msg:pose-val distance-lo) 11) 2)))))

(defun z-distance (lo-1 lo-2)
  "Returns the distance between 2 jlos in world z (lo-1.z - lo-2.z)."
  (let ((world-lo-1 (frame-query "/map" lo-1))
        (world-lo-2 (frame-query "/map" lo-2)))
    (- (aref (vision_msgs-msg:pose-val world-lo-1) 11)
       (aref (vision_msgs-msg:pose-val world-lo-2) 11))))
