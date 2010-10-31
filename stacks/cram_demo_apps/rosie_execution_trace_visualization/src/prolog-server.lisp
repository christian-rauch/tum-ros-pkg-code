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

(in-package :vis-et)

(defun json-prolog-init ()
  (json-prolog:start-prolog-server *ros-node-name* :package (find-package :vis-et)))

(register-ros-init-function json-prolog-init)

(defun pose-stamped->msg (ps)
  (roslisp:make-message "geometry_msgs/PoseStamped"
                        (frame_id header) (cl-tf:frame-id ps)
                        (stamp header) (cl-tf:stamp ps)
                        (x position pose) (cl-transforms:x (cl-transforms:origin ps))
                        (y position pose) (cl-transforms:y (cl-transforms:origin ps))
                        (z position pose) (cl-transforms:z (cl-transforms:origin ps))
                        (x orientation pose) (cl-transforms:x (cl-transforms:orientation ps))
                        (y orientation pose) (cl-transforms:y (cl-transforms:orientation ps))
                        (z orientation pose) (cl-transforms:z (cl-transforms:orientation ps))
                        (w orientation pose) (cl-transforms:w (cl-transforms:orientation ps))))

(roslisp:def-service-callback rosie_execution_trace_visualization-srv:QueryManipulationInfo (execution_trace_filename)
  (handler-case
      (flet ((make-plan-info (bdgs)
               (map 'vector (lambda (bdg)
                              (with-vars-bound
                                  (?obj-type ?obj-loc ?robot-loc ?outcome)
                                  bdg
                                (when (and ?obj-type ?obj-loc ?robot-loc ?outcome)
                                  (roslisp:make-message "rosie_execution_trace_visualization/PlanInfo"
                                                        obj_type (symbol-name ?obj-type)
                                                        obj_location (pose-stamped->msg (reference ?obj-loc))
                                                        robot_location (pose-stamped->msg  (reference ?robot-loc))
                                                        status (symbol-name ?outcome)))))
                    bdgs)))
                  
        (cet:with-offline-episode-knowledge execution_trace_filename
          (roslisp:make-response :perceives (make-plan-info
                                             (force-ll
                                              (prolog `(perceive-metadata (?_ ?obj-type ?_ ?obj-loc
                                                                              ?robot-loc ?outcome)))))
                                 :pick_ups (make-plan-info
                                            (force-ll
                                             (prolog `(pick-up-metadata (?_ ?obj-type ?_ ?obj-loc
                                                                            ?robot-loc ?outcome)))))
                                 :put_downs (make-plan-info
                                             (force-ll
                                              (prolog `(put-down-metadata (?_ ?obj-type ?_ ?obj-loc
                                                                              ?robot-loc ?outcome))))))))
    (error (e)
      (roslisp:ros-warn (query-manipulation-info rosie-executive) "Service callback failed: ~a" e)
      (roslisp:make-response))))

(roslisp:def-service-callback rosie_execution_trace_visualization-srv:ListExecutionTraces ()
  (handler-case
      (roslisp:make-response
       :filenames (map 'vector #'namestring
                       (directory (merge-pathnames
                                   (make-pathname :type "ek" :name :wild)
                                   (make-pathname
                                    :directory (roslisp:get-param "~execution_trace_dir"))))))
    (error (e)
      (roslisp:ros-warn (list-execution-traces rosie-executive) "Service callback failed: ~a" e)
      (roslisp:make-response))))

(defun init-pick-and-place-info-srv ()
  (roslisp:register-service "~query_manipulation_info" rosie_execution_trace_visualization-srv:QueryManipulationInfo)
  (roslisp:register-service "~list_exectuon_traces" rosie_execution_trace_visualization-srv:ListExecutionTraces))

(cram-roslisp-common:register-ros-init-function init-pick-and-place-info-srv)

(defun run ()
  (unwind-protect
       (progn
         (startup-ros :name "execution_trace" :anonymous nil)
         (spin-until nil 100))
    (shutdown-ros)))
