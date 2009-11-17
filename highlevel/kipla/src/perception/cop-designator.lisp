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

(defstruct cop-desig-query-info
  object-classes
  object-ids
  poses
  (matches 1) ;Number of objects to match maximally
  )

(defstruct cop-desig-location-info
  poses)

(defstruct cop-desig-info
  designator
  query
  location)

(defun ensure-desig-info (info desig)
  (or info (make-cop-desig-info
              :query (make-cop-desig-query-info)
              :location (make-cop-desig-location-info)
              :designator desig)))

;; object_classes: Cluster, IceTea, Mug, red, black, Jug
(register-object-desig-resolver type :cop (result desig)
  (with-desig-props (type) desig
    (let ((desig-info (ensure-desig-info result desig)))
      (when type
        (push (ccase type
                (mug "Mug")
                (icetea "IceTea")
                (cluster "Cluster")
                (jug "Jug")
                (placemat "PlaceMat")
                (coke "Coke"))
              (cop-desig-query-info-object-classes (cop-desig-info-query desig-info))))
      desig-info)))

(register-object-desig-resolver color :cop (result desig)
  (with-desig-props (color) desig
    (let ((desig-info (ensure-desig-info result desig)))
      (when color
        (push (ccase color
                (black "black")
                (red "red"))
              (cop-desig-query-info-object-classes (cop-desig-info-query desig-info))))
      desig-info)))

(register-object-desig-resolver at :cop (result desig)
  ;; (at <loc-desig>). loc-desig must be a resolved location
  ;; designator, i.e. it must return a valid pose (as lo id)
  (with-desig-props (at) desig
    (let ((desig-info (ensure-desig-info result desig)))
      (when at
        (setf (cop-desig-location-info-poses (cop-desig-info-location desig-info))
              (loop with loc = at
                 while loc                   
                 collecting (reference loc)
                 do (setf loc (next-solution loc)))))
      desig-info)))

(register-object-desig-resolver matches :cop (result desig)
  (with-desig-props (matches) desig
    (let ((desig-info (ensure-desig-info result desig)))
      (when matches
        (setf (cop-desig-query-info-matches (cop-desig-info-query desig-info))
              matches))
      desig-info)))
