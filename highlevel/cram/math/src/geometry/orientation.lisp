
(in-package :cma)

(defclass quaternion ()
  ((w :initform 1.0 :initarg :w :accessor w)
   (x :initform 0.0 :initarg :x :accessor x)
   (y :initform 0.0 :initarg :y :accessor y)
   (z :initform 0.0 :initarg :z :accessor z) )
  (:documentation "a three-dimensional orientation described as quaternion"))

(defclass orientation-euler ()
  ((ax :initform 0.0 :initarg :ax :accessor ax)
   (ay :initform 0.0 :initarg :ay :accessor ay)
   (az :initform 0.0 :initarg :az :accessor az))
  (:documentation "a three-dimensional euler orientation (rotation: z-y-x)"))

(defun quaternion (&key (x 0.0) (y 0.0) (z 0.0) (w 1.0))
  (make-instance 'quaternion :x x :y y :z z :w w))

(defmethod print-object ((obj quaternion) strm)
  (print-unreadable-object (obj strm :type t :identity t)
    (with-slots (x y z w) obj
      (format strm "X: ~a Y: ~a Z: ~a W: ~a" x y z w))))

(defmethod print-object ((obj orientation-euler) strm)
  (print-unreadable-object (obj strm :type t :identity t)
    (with-slots (ax ay az) obj 
      (format strm "AX: ~a AY: ~a AZ: ~a" ax ay az))))

(defun quaternion-magnitude (q)
  "compute quaternion magnitude"
  (sqrt (+ (expt (w q) 2)
           (expt (x q) 2)
           (expt (y q) 2)
           (expt (z q) 2))))

(defun quaternion-inverse (q)
  (make-instance 'quaternion
                 :w (w q)
                 :x (- (x q))
                 :y (- (y q))
                 :z (- (z q))))

(defun quaternion-normalize (q)
  (let ((magnitude (quaternion-magnitude q)))
    (with-slots (w x y z) q
      (make-instance 'quaternion
                     :w (/ w magnitude)
                     :x (/ x magnitude)
                     :y (/ y magnitude)
                     :z (/ z magnitude)))))

(defun quaternion-* (&rest rotations)
  "Multiply quaternions. The rotations are processed from right to
   left."
  (flet ((quat-*-1 (q2 q1)
           (let ((q1w (w q1))
                 (q1x (x q1))
                 (q1y (y q1))
                 (q1z (z q1))
                 (q2w (w q2))
                 (q2x (x q2))
                 (q2y (y q2))
                 (q2z (z q2)))
             (let ((resw (- (* q1w q2w) (* q1x q2x) (* q1y q2y) (* q1z q2z)))
                   (resx (+ (* q1w q2x) (* q1x q2w) (* q1y q2z) (* -1 q1z q2y)))
                   (resy (+ (* q1w q2y) (* -1 q1x q2z) (* q1y q2w) (* q1z q2x)))
                   (resz (+ (* q1w q2z) (* q1x q2y) (* -1 q1y q2x) (* q1z q2w))))
               (setf (w q2) resw)
               (setf (x q2) resx)
               (setf (y q2) resy)
               (setf (z q2) resz)))
           q2))
    (reduce #'quat-*-1 (reverse rotations) :initial-value (make-instance 'quaternion))))
