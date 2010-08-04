;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2010 by Lorenz Moesenlechner <moesenle@cs.tum.edu>
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

(defclass location-cost-map ()
  ((width :initarg :width :reader width)
   (height :initarg :height :reader height)
   (origin-x :initarg :origin-x :reader origin-x :initform 0.0)
   (origin-y :initarg :origin-y :reader origin-y :initform 0.0)
   (resolution :initarg :resolution :reader resolution :initform 0.01
               :documentation "The resolution of the cost map,
               i.e. the step size used when evaluating the cost
               functions.")
   (cost-map)
   (cost-functions :reader cost-functions :initarg :cost-functions
                   :initform nil
                   :documentation "Sequence of closures that take an a
                                   x and a y coordinate and return the
                                   corresponding cost in the interval
                                   [0;1]")))

(defgeneric get-cost-map (map)
  (:documentation "Returns the costmap as a two-dimensional array of
  FLOAT."))

(defgeneric get-map-value (map &key x y)
  (:documentation "Returns the cost of a specific pose."))

(defgeneric register-cost-function (map fun &optional score)
  (:documentation "Registers a new cost function. The optional
  argument `score' allows to define an order in which to apply each
  cost function. Higher scores are evaluated first."))

(defmethod get-cost-map ((map location-cost-map))
  (flet ((calculate-map-value (map x y)
           (reduce (lambda (prev-value cost-function)
                     ;; (format t "prev-value: ~a cost-function ~a~%" prev-value cost-function)
                     (when (eql prev-value 0.0)
                       (return-from calculate-map-value 0.0))
                     (* prev-value (funcall (car cost-function) x y)))
                   (cost-functions map)
                   :initial-value 1.0))
         (normalize-map (map sum)
           (declare (type float sum))
           (let ((1d-map (make-array (* (array-dimension map 0) (array-dimension map 1))
                                     :element-type 'float
                                     :displaced-to map)))
             (map-into 1d-map (lambda (val) (/ val sum)) 1d-map))))
    
    (with-slots (width height origin-x origin-y resolution) map
      (check-type width integer)
      (check-type height integer)      
      (unless (slot-boundp map 'cost-map)
        (setf (slot-value map 'cost-functions)
              (sort (slot-value map 'cost-functions) #'> :key #'cdr))
        (let ((new-cost-map (make-array `(,(round (/ width resolution))
                                          ,(round (/ height resolution)))
                                        :element-type 'float))
              (sum 0.0))
          (loop for y from 0 below (array-dimension new-cost-map 1) do
            (loop for x from 0 below (array-dimension new-cost-map 1) do
              (let ((cost-value (calculate-map-value
                                 map
                                 (+ (* x resolution) origin-x)
                                 (+ (* y resolution) origin-y))))
                (setf sum (+ sum cost-value))
                (setf (aref new-cost-map x y) cost-value))))
          (normalize-map new-cost-map sum)
          (setf (slot-value map 'cost-map) new-cost-map)))
      (slot-value map 'cost-map))))

(defmethod get-map-value ((map location-cost-map) &key x y)
  (aref (get-cost-map map)
        (/ x (slot-value map 'resolution))
        (/ y (slot-value map 'resolution))))

(defmethod register-cost-function ((map location-cost-map) fun &optional (score 0))
  (push (cons fun score) (slot-value map 'cost-functions)))
