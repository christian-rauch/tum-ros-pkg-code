
(in-package :kipla-reasoning)

(deftype occupancy-grid-data () '(simple-array fixnum 2))

(defclass occupancy-grid-metadata ()
  ((width :reader width :initarg :width :type double-float)
   (height :reader height :initarg :height :type double-float)
   (origin-x :reader origin-x :initarg :origin-x :type double-float)
   (origin-y :reader origin-y :initarg :origin-y :type double-float)
   (resolution :reader resolution :initarg :resolution :type double-float)))

(defclass occupancy-grid (occupancy-grid-metadata)
  ((grid :reader grid :type (simple-array fixnum 2)))
  (:documentation "Defines an occupancy grid. Please note that in
  contras to ros' OccupancyGrid message, this grid only contains
  values of 0 or 1."))

(defmethod initialize-instance :after ((grid occupancy-grid) &key)
  (with-slots (width height resolution grid) grid
    (setf grid (make-array (list (round (/ height resolution))
                                 (round (/ width resolution)))
                           :element-type 'fixnum
                           :initial-element 0))))

(defun make-occupancy-grid (width height origin-x origin-y resolution &optional initial-data)
  (check-type initial-data (or null (array * *)))
  (let ((grid (make-instance 'occupancy-grid
                             :width width :height height
                             :origin-x origin-x :origin-y origin-y
                             :resolution resolution)))
    (when initial-data
      (map-into (make-array (array-total-size (grid grid))
                            :element-type 'fixnum :displaced-to (grid grid))
                #'identity (make-array (array-total-size initial-data)
                                       :element-type (array-element-type initial-data)
                                       :displaced-to initial-data)))
    grid))

(defun copy-occupancy-grid (src)
  (make-occupancy-grid (width src) (height src)
                       (origin-x src) (origin-y src)
                       (resolution src) (grid src)))

(defun invert-occupancy-grid (src)
  (let* ((result (make-occupancy-grid (width src) (height src) (origin-x src) (origin-y src)
                                      (resolution src)))
         (src-grid (grid src))
         (result-grid (grid result)))
    (declare (type occupancy-grid-data src-grid result-grid))
    (dotimes (row (array-dimension src-grid 0))
      (dotimes (col (array-dimension src-grid 1))
        (setf (aref result-grid row col)
              (if (eql (aref src-grid row col) 0) 1 0))))
    result))

(defun set-grid-cell (grid x y)
  (let ((grid-arr (grid grid)))
    (declare (type occupancy-grid-data grid-arr))
    (setf (aref grid-arr
                (round (/ (- y (origin-y grid)) (resolution grid)))
                (round (/ (- x (origin-x grid)) (resolution grid))))
          1)))

(defun clear-grid-cell (grid x y)
  (let ((grid-arr (grid grid)))
    (declare (type occupancy-grid-data grid-arr))
    (setf (aref grid-arr
                (round (/ (- y (origin-y grid)) (resolution grid)))
                (round (/ (- x (origin-x grid)) (resolution grid))))
          0)))

(defun get-grid-cell (grid x y)
  (let ((grid-arr (grid grid)))
    (declare (type occupancy-grid-data grid-arr))
    (aref grid-arr
          (round (/ (- y (origin-y grid)) (resolution grid)))
          (round (/ (- x (origin-x grid)) (resolution grid))))))