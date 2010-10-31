
(in-package :table-costmap)

(defvar *table-clusters* nil
  "List of TABLE-CLUSTERs that contain the current data about
  tables.")

(defvar *initial-stddev* 1.0d0
  "Initial std deviation for table-clusters.")

(defvar *accept-threshold* 0.1
  "Threshold above which a point is accepted to belong to a specific
  table.")

(defclass table-cluster ()
  ((mean :reader mean :initarg :mean)
   (cov :reader cov :initarg :cov
        :initform (make-array
                   '(3 3)
                   :element-type 'double-float
                   :initial-contents `((,*initial-stddev* 0.0d0 0.0d0)
                                       (0.0d0 ,*initial-stddev* 0.0d0)
                                       (0.0d0 0.0d0 ,*initial-stddev*))))
   (name :reader name :initarg :name)))

(defmethod initialize-instance :after ((props table-cluster) &key)
  (with-slots (name) props
    (unless (slot-boundp props 'mean)
      (setf (slot-value props 'mean)
            (get-annotated-point name)))))

(defun 3d-vector->mean (vec)
  (let ((result (make-array 2 :element-type 'double-float)))
    (declare (type (simple-array double-float (2)) result))
    (setf (aref result 0) (cl-transforms:x vec)
          (aref result 1) (cl-transforms:y vec))
    result))

(defun 2d-cov (cov)
  (let ((result (make-array '(2 2) :element-type (array-element-type cov))))
    (declare (type (simple-array * 2) cov)
             (type (simple-array * (2 2)) result))
    (dotimes (y 2)
      (dotimes (x 2)
        (setf (aref result y x) (aref cov y x))))
    result))

(defun points-mean (pts)
  "Returns the mean vector of all points in `pts'"
  (let ((x 0.0d0)
        (y 0.0d0)
        (z 0.0d0)
        (n 0))
    (map 'nil (lambda (pt)
                (incf n)
                (incf x (cl-transforms:x pt))
                (incf y (cl-transforms:y pt))
                (incf z (cl-transforms:z pt)))
         pts)
    (cl-transforms:make-3d-vector (/ x n)
                                  (/ y n)
                                  (/ z n))))

(defun points-cov (pts &optional (mean (points-mean pts)))
  "Returns the covariance of `pts'."
  (let ((cov (make-array
              '(3 3)
              :element-type 'double-float
              :initial-element 0.0d0))
        (n 0)
        (accessors (make-array 3 :initial-contents (list #'cl-transforms:x
                                                         #'cl-transforms:y
                                                         #'cl-transforms:z))))
    (declare (type (simple-array double-float (3 3)) cov)
             (type (simple-array t (3)) accessors))
    (map 'nil (lambda (pt)
                (dotimes (y 3)
                  (dotimes (x (+ y 1))
                    (let ((val (* (- (funcall (aref accessors x) pt)
                                     (funcall (aref accessors x) mean))
                                  (- (funcall (aref accessors y) pt)
                                     (funcall (aref accessors y) mean)))))
                      (incf (aref cov y x) val))))
                (incf n))
         pts)
    (dotimes (y 3)
      (dotimes (x (+ 1 y))
        (setf (aref cov y x) (/ (aref cov y x) n))
        (unless (eql x y)
          (setf (aref cov x y) (aref cov y x)))))
    cov))

(defun cluster-tables-iterate (properties msg)
  "Takes a grid cells message and returns the updated list of
  TABLE-CLUSTER. `properties' is the list of TABLE-CLUSTER to be
  updated."
  (labels ((find-closest-table (pt table-props &optional
                                   (best (car table-props))
                                   (best-dist (cl-transforms:v-dist
                                               pt (mean best))))
             (if (not table-props)
                 best
                 (let ((dist (cl-transforms:v-dist
                              pt (mean (car table-props)))))
                   (if (< dist best-dist)
                       (find-closest-table pt (cdr table-props) (car table-props) dist)
                       (find-closest-table pt (cdr table-props) best best-dist)))))
           (cluster-points (points gaussians)
             (let ((annotated-points (make-hash-table)))
               (map 'nil
                    (lambda (pt)
                      (let ((pt-3d (cl-transforms:make-3d-vector
                                    (geometry_msgs-msg:x-val pt)
                                    (geometry_msgs-msg:y-val pt)
                                    (geometry_msgs-msg:z-val pt))))
                        (let ((closest (find-closest-table
                                        pt-3d properties)))
                          ;; (push pt-3d (gethash (name closest) annotated-points))
                          (when (> (funcall (gethash (name closest) gaussians)
                                            (cl-transforms:x pt-3d)
                                            (cl-transforms:y pt-3d))
                                   *accept-threshold*)
                            (push pt-3d (gethash (name closest) annotated-points)))
                          )))
                    points)
               annotated-points)))
    (let ((gaussians (make-hash-table)))
      (dolist (prop properties)
        (with-slots (mean cov name) prop
          (setf (gethash name gaussians)
                (make-gauss-cost-function (3d-vector->mean mean)
                                          (2d-cov cov)))))
      (with-fields (cells) msg
        (let ((clustered-points (cluster-points cells gaussians)))
          (mapcar
           (lambda (prop)
             (if (gethash (name prop) clustered-points)
                 (let* ((mean (points-mean (gethash (name prop)
                                                    clustered-points)))
                        (cov (points-cov (gethash (name prop)
                                                  clustered-points)
                                         mean)))
                   (make-instance 'table-cluster
                                  :name (name prop)
                                  :mean mean
                                  :cov cov))
                 prop))
           properties))))))

(defun init-table-clusters ()
  (setf *table-clusters* (mapcar (alexandria:curry #'make-instance 'table-cluster :name)
                                 (get-annotation-names))))

(defun cluster-tables (table-clusters msg &key
                       (max-iters 10)
                       (mean-threshold 0.001)
                       (cov-threshold 0.001))
  (labels ((cov-equal-p (cov-1 cov-2 threshold)
             (dotimes (y 3)
               (dotimes (x (1+ y))
                 (when (> (abs (- (aref cov-1 y x) (aref cov-2 y x)))
                          threshold)
                   (return-from cov-equal-p nil))))
             t)
           (mean-equal-p (mean-1 mean-2 threshold)
             (< (cl-transforms:v-dist mean-1 mean-2) threshold))
           (table-cluster-equal-p (prop-1 prop-2 mean-threshold cov-threshold)
             (and (mean-equal-p (mean prop-1) (mean prop-2) mean-threshold)
                  (cov-equal-p (cov prop-1) (cov prop-2) cov-threshold)))
           (solutions-equal-p (solution-1 solution-2 mean-threshold cov-threshold)
             (every (alexandria:rcurry #'table-cluster-equal-p mean-threshold cov-threshold)
                    solution-1 solution-2)))
    (let ((new-solution (cluster-tables-iterate table-clusters msg)))
      (if (or (solutions-equal-p table-clusters new-solution mean-threshold cov-threshold)
              (eql max-iters 0))
          new-solution
          (cluster-tables new-solution msg
                          :max-iters (1- max-iters)
                          :mean-threshold mean-threshold
                          :cov-threshold cov-threshold)))))

(defun get-table-cluster (name)
  (find (symbol-name name) *table-clusters*
        :key (alexandria:compose #'symbol-name #'name)
        :test #'equal))
