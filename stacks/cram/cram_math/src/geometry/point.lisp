
(in-package :cma)

(defclass point ()
  ((x :initform 0.0 :initarg :x :accessor x)
   (y :initform 0.0 :initarg :y :accessor y)
   (z :initform 0.0 :initarg :z :accessor z))
  (:documentation "a three-dimensional point"))

(defun point (&key (x 0.0) (y 0.0) (z 0.0))
  (assert (typep x 'number))
  (assert (typep y 'number))
  (assert (typep z 'number))  
  (make-instance 'point :x x :y y :z z))

(defmethod print-object ((obj point) strm)
  (print-unreadable-object (obj strm :type t :identity t)
    (with-slots (x y z) obj
      (format strm "X: ~a Y: ~a Z: ~a" x y z))))

(defun euclidean-distance (p1 p2)
  (sqrt (+ (expt (- (x p1) (x p2)) 2)
           (expt (- (y p1) (y p2)) 2)
           (expt (- (z p1) (z p2)) 2))))

(defun point-magnitude (p)
  "compute point magnitude"
  (sqrt (+ (expt (x p) 2)
           (expt (y p) 2)
           (expt (z p) 2))))

(defun dot-product (p1 p2)
  (+ (* (x p1) (x p2))
     (* (y p1) (y p2)
        (z p1) (z p2))))

(defun cross-product (p1 p2)
  (make-instance '3d-point
    :x (- (* (y p1) (z p2))
          (* (z p1) (y p2)))
    :y (- (* (z p1) (x p2))
          (* (x p1) (z p2)))
    :z (- (* (x p1) (y p2))
          (* (y p1) (x p2)))))

(defun point-inverse (p)
  (with-slots (x y z) p
    (make-instance 'point
                   :x (- x)
                   :y (- y)
                   :z (- z))))

(defun point-+ (p1 &rest points)
  (reduce (lambda (result p)
            (setf (x result) (+ (x result) (x p))
                  (y result) (+ (y result) (y p))
                  (z result) (+ (z result) (z p)))
            result)
          points
          :initial-value (make-instance 'point :x (x p1) :y (y p1) :z (z p1))))
