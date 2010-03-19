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

(defvar *cop-output-queue* (make-fluent :name '*cop-output-queue* :value nil))
(defvar *cop-parameters* (make-hash-table :test 'cl:eq))

(defclass cop-perceived-object (perceived-object)
  ((object-id :initarg :object-id :accessor perceived-object-id)
   (classes :initarg :classes
            :accessor perceived-object-classes)))

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

(defun cop-query (query-info &key (command :localize))
  "Executes a cop query.  `command' can be :localize, :track, :refine,
   :prove, :stop-track, :start-attend or :stop-attend."
  (wait-for-service "/cop/in")
  (call-service "/cop/in" 'vision_srvs-srv:cop_call
                :outputtopic "/kipla/cop_reply"
                :object_classes (make-array (list-length (cop-desig-query-info-object-classes query-info))
                                            :initial-contents (cop-desig-query-info-object-classes query-info))
                :object_ids (make-array (list-length (cop-desig-query-info-object-ids query-info))
                                        :initial-contents (cop-desig-query-info-object-ids query-info))
                :action_type (ecase command
                               (:localize #x0000)
                               (:track #x0100)
                               (:refine #x0300)
                               (:prove #x0400)
                               (:stop-track #x0800)
                               (:start-attend #x1600)
                               (:stop-attend #x3200))
                :number_of_objects (cop-desig-query-info-matches query-info)
                :list_of_poses (make-array (list-length (cop-desig-query-info-poses query-info))
                                           :initial-contents (mapcar (lambda (id)
                                                                       (make-instance 'vision_msgs-msg:<apriori_position>
                                                                         :probability 0.9
                                                                         :positionid (jlo:id id)))
                                                                     (cop-desig-query-info-poses query-info))))
  (wait-for *cop-output-queue* :handle-missed-pulses :never)
  (pop (value *cop-output-queue*)))

(defun cop-class->property (class)
  "Takes a cop class as a string and returns the corresponding
   designator tuple."
  (string-case class
    ;; Types
    ("Mug" '(type mug))
    ("IceTes" '(type icetea))
    ("Cluster" '(type cluster))
    ("Jug" '(type jug))
    ("PlaceMat" '(type placemat))
    ("Coke" '(type coke))
    ;; Colors
    ("red" '(color red))
    ("white" '(color white))
    ("black" '(color black))
    ("blue" '(color blue))
    ("green" '(color green))))


(defmethod perceived-object-properties ((perceived-object cop-perceived-object))
  (remove nil (mapcar #'cop-class->property
                      (perceived-object-classes perceived-object))))

(defun cop-reply->perceived-object (reply)
  (make-instance 'cop-perceived-object
                 :pose (jlo:make-jlo :id (vision_msgs-msg:position-val reply))
                 :object-id (vision_msgs-msg:objectid-val reply)
;                 :classes (remove-duplicates (map 'list #'identity (vision_msgs-msg:classes-val reply))
;                                             :test #'equal)
                 :classes (remove-duplicates (map 'list #'vision_msgs-msg:sem_class-val (vision_msgs-msg:models-val reply))
                                             :test #'equal)
                 :probability (vision_msgs-msg:probability-val reply)))

;; (defun look-at (pose-id)
;;   (publish "/head_controller/lookat_lo" (make-instance 'std_msgs-msg:<uint64> :data pose-id)))

(def-actionlib-interface ptu "ptu"
  'cogman_msgs-msg:<ptuaction>)

(register-ros-init-function ptu-init)
(register-ros-cleanup-function ptu-cleanup)

(defun look-at (jlo)
  (assert (typep jlo 'jlo::jlo))
  ;; Note: we seem to have an evil race condition here. When taking
  ;; out the sleep, we get failures.
  (log-msg :info "Looking at ~a" jlo)
  (unless (member :ptu *kipla-features*)
    (sleep 0.5)
    (return-from look-at nil))
  (ptu-send-goal (make-instance 'cogman_msgs-msg:<ptugoal>
                                :lo_id (jlo:id jlo)))
  (sleep 0.5))
