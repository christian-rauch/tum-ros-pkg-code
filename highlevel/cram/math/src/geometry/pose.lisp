
(in-package :cma)

(defclass pose ()
  ((position :initform (make-instance 'point) :initarg :position
             :accessor pos)
   (orientation :initform (make-instance 'quaternion) :initarg :orientation
                :accessor orientation))
  (:documentation "a three-dimensional pose"))

(defun pose (&optional (position (point)) (orientation (quaternion)))
  (make-instance 'pose
                 :position position
                 :orientation orientation))

(defun pose-inverse (p)
  (let ((q-inv (quaternion-inverse (orientation p))))
    (make-instance 'pose
                   :position (rotate-point (point-inverse (pos p)) q-inv)
                   :orientation q-inv)))

(defun pose-* (&rest poses)
  "Compose poses by first rotating around the parent pose and then adding up the points.
   Processes from right to left, i.e. the right-most transformation
   pair of transformations is applied first."
  (reduce (lambda (prev p)
            (make-instance 'pose
                           :position (point-+ (pos p) (rotate-point (pos prev) (orientation p)))
                           :orientation (quaternion-* (orientation prev) (orientation p))))
          (reverse poses)))

(defun pose->matrix (pose)
  "Constructs a homogenous matrix from pose and returns it as a 2D array.
   Please note: to convert it to a 1D array, you can do the following:

  (make-array (array-total-size x) 
              :element-type (array-element-type x) 
              :displaced-to x)"
  (with-slots (position orientation) pose
    (let ((result (make-array '(4 4) :initial-element 0.0)))
      (with-slots (x y z w) orientation
        (setf (aref result 0 0) (- 1 (* 2 y y) (* 2 z z))
              (aref result 0 1) (- (* 2 x y) (* 2 z w))
              (aref result 0 2) (+ (* 2 x z) (* 2 y w))
              (aref result 1 0) (+ (* 2 x y) (* 2 z w))
              (aref result 1 1) (- 1 (* 2 x x) (* 2 z z))
              (aref result 1 2) (- (* 2 y z) (* 2 x w))
              (aref result 2 0) (- (* 2 x y) (* 2 y w))
              (aref result 2 1) (+ (* 2 y z) (* 2 x w))
              (aref result 2 2) (- 1 (* 2 x x) (* 2 y y))))
      (with-slots (x y z) position
        (setf (aref result 0 3) x
              (aref result 1 3) y
              (aref result 2 3) z
              (aref result 3 3) 1))
      result)))

(defmethod print-object ((obj pose) strm)
  (print-unreadable-object (obj strm :type t)
    (with-slots (position orientation) obj
      (format strm "~{~<~%   ~a~>~}" (list position orientation)))))
