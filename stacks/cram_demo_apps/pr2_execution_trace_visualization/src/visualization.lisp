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

(defvar *marker-id-counter* 0)
(defvar *marker-pub* nil)

(defun init-pick-and-place-visualization ()
  (setf *marker-pub* (advertise
                      "~pick_and_place_markers"
                      "visualization_msgs/Marker")))

(register-ros-init-function init-pick-and-place-visualization)

(defun clear-markers ()
  (dotimes (i *marker-id-counter*)
    (publish *marker-pub* (make-message
                           "visualization_msgs/Marker"
                           (frame_id header) "/map"
                           ns "pick_and_place"
                           id i
                           action (symbol-code 'visualization_msgs-msg:<marker> :delete))))
  (setf *marker-id-counter* 0))

(defun make-color-msg (color &optional (alpha 1.0))
  (ecase color
    (:red (make-message "std_msgs/ColorRGBA"
                        r 1
                        g 0
                        b 0
                        a alpha))
    (:green (make-message "std_msgs/ColorRGBA"
                          r 0
                          g 1
                          b 0
                          a alpha))
    (:yellow (make-message "std_msgs/ColorRGBA"
                           r 1
                           g 1
                           b 0
                           a alpha))))

(defun status-color (status &optional (alpha 1.0))
  (make-color-msg (case status
                    (:succeeded :green)
                    (:failed :red)
                    (t :yellow))
                  alpha))

(defun visualize-robot-pose (pose status)
  "Publishes a flat sphere at pose. When `status' is :SUCCEDED, its
  color is green, when it is :FAILED, its color is red and yellow
  otherwise."
  (publish
   *marker-pub*
   (make-message "visualization_msgs/Marker"
                 (frame_id header) (tf:frame-id pose)
                 ns "pick_and_place"
                 id (prog1 *marker-id-counter* (incf *marker-id-counter*))
                 type (symbol-code 'visualization_msgs-msg:<marker> :sphere)
                 action (symbol-code 'visualization_msgs-msg:<marker> :add)
                 (x position pose) (cl-transforms:x
                                    (cl-transforms:origin pose))
                 (y position pose) (cl-transforms:y
                                    (cl-transforms:origin pose))
                 (z position pose) (cl-transforms:z
                                    (cl-transforms:origin pose))
                 (x orientation pose) 0
                 (y orientation pose) 0
                 (z orientation pose) 0
                 (w orientation pose) 1
                 (x scale) 0.75
                 (y scale) 0.75
                 (z scale) 0.10
                 color (status-color status 0.5))))

(defun visualize-tf-trajectory (frame start-time end-time status)
  (unless (roslisp:wait-for-service "/tf_bag_trajectory_visualization/visualize_trajectory" 0.5)
    (ros-warn (visualization pr2-executive)
              "Could not find service '/tf_bag_trajectory_visualization/visualize_trajectory'")
    (return-from visualize-tf-trajectory nil))
  (roslisp:with-fields (points)
      (roslisp:call-service "/tf_bag_trajectory_visualization/visualize_trajectory"
                            "tf_trajectory_visualization/VisualizeTFTrajectory"
                            :start_time start-time
                            :end_time end-time
                            :tf_frame frame
                            :publish nil)
    (publish
     *marker-pub*
     (make-message "visualization_msgs/Marker"
                   (frame_id header) "/map"
                   ns "pick_and_place"
                   id (prog1 *marker-id-counter* (incf *marker-id-counter*))
                   type (symbol-code 'visualization_msgs-msg:<marker> :line_strip)
                   action (symbol-code 'visualization_msgs-msg:<marker> :add)
                   (x position pose) 0
                   (y position pose) 0
                   (z position pose) 0
                   (x orientation pose) 0
                   (y orientation pose) 0
                   (z orientation pose) 0
                   (w orientation pose) 1
                   (x scale) 0.05
                   (y scale) 0
                   (z scale) 0
                   color (status-color status)
                   points points))))

(defun visualize-object (type pose)
  (let ((type (symbol-code 'visualization_msgs-msg:<marker>
                                   (case type
                                     (assam-blend :cube)
                                     (mug :cylinder)
                                     (cluster :sphere)
                                     (round-plate :sphere)
                                     (bottle :cylinder)
                                     (t :cube))))
        (scale (case type
                 (round-plate '(0.20 0.20 0.02))
                 (bottle '(0.07 0.07 0.20))
                 (t '(0.1 0.1 0.1)))))
    (publish
     *marker-pub*
     (make-message "visualization_msgs/Marker"
                   (frame_id header) "/map"
                   ns "pick_and_place"
                   id (prog1 *marker-id-counter* (incf *marker-id-counter*))
                   type type
                   action (symbol-code 'visualization_msgs-msg:<marker> :add)
                   (x position pose) (cl-transforms:x
                                      (cl-transforms:origin pose))
                   (y position pose) (cl-transforms:y
                                      (cl-transforms:origin pose))
                   (z position pose) (cl-transforms:z
                                      (cl-transforms:origin pose))
                   (x orientation pose) (cl-transforms:x
                                         (cl-transforms:orientation pose))
                   (y orientation pose) (cl-transforms:y
                                         (cl-transforms:orientation pose))
                   (z orientation pose) (cl-transforms:z
                                         (cl-transforms:orientation pose))
                   (w orientation pose) (cl-transforms:w
                                         (cl-transforms:orientation pose))
                   (x scale) (first scale)
                   (y scale) (second scale)
                   (z scale) (third scale)
                   (r color) 0
                   (g color) 0
                   (b color) 1
                   (a color) 1))))
