
(in-package :cma)

(defun quaternion->point (quat)
  "convert a quaternion to a point"
  (make-instance 'point
                 :x (x quat)
                 :y (y quat)
                 :z (z quat)))

(defun point->quaternion (p)
  (make-instance 'quaternion
                 :x (x p)
                 :y (y p)
                 :z (z p)))

(defun axis+angle->quaternion (p a)
  "create a quaternion from an axis and angle"
  (let ((mag (point-magnitude p)))
    (cond ((> mag 0)
           (let ( (angle (* a 0.5)) )
             (let ((scale (/ (sin angle) mag)))
               (make-instance 'quaternion
                              :w (cos angle)
                              :x (* (x p) scale)
                              :y (* (y p) scale)
                              :z (* (z p) scale)))))
          (t
           (make-instance 'quaternion)))))

(defun quaternion->axis+angle (q)
  "convert quaterion to axis and angle"
  (values
    (make-instance 'point
                   :x (x q)
                   :y (y q)
                   :z (z q))
    (* 2 (acos (w q)))))

(defun euler->quaternion (&key (ax 0.0) (ay 0.0) (az 0.0))
  "create a quaternion from euler angles"
  (let ((phi (* ax 0.5))
        (the (* ay 0.5))
        (psi (* az 0.5)))
    (make-instance 'quaternion
                   :w (+ (* (cos phi) (cos the) (cos psi)) (* (sin phi) (sin the) (sin psi)))
                   :x (- (* (sin phi) (cos the) (cos psi)) (* (cos phi) (sin the) (sin psi)))
                   :y (+ (* (cos phi) (sin the) (cos psi)) (* (sin phi) (cos the) (sin psi)))
                   :z (- (* (cos phi) (cos the) (sin psi)) (* (sin phi) (sin the) (cos psi))))))

(defun quaternion->euler (q)
  "create a quaternion to euler angles"
  (with-slots (w x y z) q
    (make-instance 'orientation-euler
                   :ax (atan (* 2 (+ (* y z) (* w x)))
                             (+ (* w w) (* -1 x x) (* -1 y y) (* z z)))
                   :ay (asin (max -1.0 (min 1.0 (* -2 (- (* x z) (* w y))))))
                   :az (atan (* 2 (+ (* x y) (* w z)))
                             (+ (* w w) (* x x) (* -1 y y) (* -1 z z))))))

(defun rotate-point (p rot)
  "Rotate a point `p' with quaternion `rot'"
  (let ((point-4d (point->quaternion p))
        (rot-inv (quaternion-inverse rot)))
    (quaternion->point
     (quaternion-* rot point-4d rot-inv))))

(defun angle-between-quaternions (q1 q2)
  "Returns two values: the angle between quaternions `q1' and `q2' and
   the rotation axis."
  (multiple-value-bind (axis angle)
      (quaternion->axis+angle (quaternion-* (quaternion-inverse q1) q2))
    (values angle axis)))

(defun angle-between-points (p1 p2)
  "Returns the angle between two points."
  (let ((prod (dot-product p1 p2)))
    (unless (= prod 0.0)
      (acos (/ prod (* (point-magnitude p1)
                       (point-magnitude p2)))))))
