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

(defun do-cop-search (desig query-info &key (command :localize))
  (let ((cop-reply (cop-query query-info :command command)))
    (when (equal (vision_msgs-msg:error-val cop-reply) "")
      (map 'list (lambda (found-pose)
                   (let ((perceived-object (cop-reply->perceived-object found-pose)))
                     (assert-perceived-object perceived-object (description desig))
                     perceived-object))
           (vision_msgs-msg:found_poses-val cop-reply)))))

(defun get-clusters ()
  (with-designators ((clusters (object '((type cluster)))))
    (execute-object-search-function clusters)))

(defun pre-process-perceived-objects (desig objects)
  (declare (ignore desig))  
  ;; The intention for this function is to pre-sort clusters before
  ;; searching for objects. That means using classification according
  ;; to desig (the most probable cluster/object matching desig should
  ;; come first)
  objects)

(defmethod object-search-function ((type (cl:eql 'cluster)) desig &optional perceived-object)
  (let ((query-info (cop-desig-info-query (resolve-object-desig desig :cop))))
    (cond ((and perceived-object
                (typep perceived-object 'cop-perceived-object))
           (setf (cop-desig-query-info-object-ids query-info)
                 (list (perceived-object-id perceived-object)))
           (setf (cop-desig-query-info-poses query-info)
                 (list (perceived-object-pose perceived-object)))
           (setf (cop-desig-query-info-object-classes query-info) nil))
          (t
           (setf (cop-desig-query-info-poses query-info)
                 (list (jlo:make-jlo :name "/sr4")))))
    (do-cop-search desig query-info)))

(defmethod object-search-function ((type (cl:eql 'object)) desig &optional perceived-object)
  (flet ((find-flat-objects ()
           ;; Not implemented yet.
           nil)
         (refine-clusters (clusters)
           (let ((query-info (make-cop-desig-query-info
                              :matches 1)))
             (mapcan (lambda (cluster)
                       (setf (cop-desig-query-info-object-ids query-info)
                             (list (perceived-object-id cluster)))
                       (setf (cop-desig-query-info-poses query-info)
                             (list (perceived-object-pose cluster)))
                       (setf (cop-desig-query-info-object-classes query-info) nil)
                       (do-cop-search desig query-info :command :refine))
                     clusters))))
    (cond (perceived-object
           ;; (call-next-method)
           nil)
          (t
           (append (find-flat-objects)
                   (refine-clusters (get-clusters)))))))

(defmethod object-search-function ((type t) desig &optional previous-object)
  ;; The default behavior is the following: If no perceived-object is
  ;; passed, first search for clusters and use the result for finding
  ;; the object. Otherwise, use `perceived-object' for it.
  (flet ((set-perceived-object-pose (query-info pose)
           (let ((result (copy-cop-desig-query-info query-info)))
             (setf (cop-desig-query-info-poses result)
                   (list pose))
             result)))
    (let ((query-info (cop-desig-info-query (resolve-object-desig desig :cop))))
      ;; When iterating over clusters, we want to find only one
      ;; object.
      (setf (cop-desig-query-info-matches query-info) 1)
      (when previous-object
        (setf (cop-desig-query-info-object-ids query-info)
              (list (perceived-object-id previous-object)))
        (setf (cop-desig-query-info-poses query-info)
              (list (perceived-object-pose previous-object))))
      (let ((perceived-objects (or (when previous-object
                                     (list previous-object))
                                   (get-clusters))))
        (setf (cop-desig-query-info-poses query-info)
              (mapcar #'perceived-object-pose
                      (pre-process-perceived-objects desig perceived-objects)))
        (mapcan (alexandria:compose (alexandria:curry #'do-cop-search desig)
                                    (alexandria:curry #'set-perceived-object-pose query-info)
                                    #'perceived-object-pose)
                perceived-objects)))))
