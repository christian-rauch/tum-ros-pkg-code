
(in-package :turtle)

(defvar *turtle-1-pose-fluent* (make-fluent :name 'turtle-1-pose))
(defvar *turtle-2-pose-fluent* (make-fluent :name 'turtle-2-pose))

(defun spawn-turtle (&key (x 0) (y 0) (theta 0))
  (turtlesim-srv:name-val
   (roslisp:call-service "/spawn" 'turtlesim-srv:spawn
                         :x x :y y :theta theta)))

(defun kill-turtle (name)
  (roslisp:call-service "/kill" 'turtlesim-srv:kill :name name))

(defun reset-turtlesim ()
  (roslisp:call-service "/reset" 'std_srvs-srv:empty))

(defun turtle-pose->pose (p)
  (pose (point :x (turtlesim-msg:x-val p)
               :y (turtlesim-msg:y-val p))
        (cma:euler->quaternion :az (turtlesim-msg:theta-val p))))

(defun calculate-cmd (goal)
  "Uses the current turtle position and calculates the actual velocity
  command."
  (let* ((current-pose (value *turtle-2-pose-fluent*))
         (goal-pose (pose goal))
         (diff-pose (pose-* (pose-inverse current-pose) goal-pose)))
    (make-instance 'turtlesim-msg:<velocity>
                   :angular (* 4 (atan (y (pos diff-pose)) (x (pos diff-pose))))
                   :linear 1.5)))

(defun turtle-update-fun (fluent)
  (lambda (pose)
    (setf (value fluent) (turtle-pose->pose pose))))

(defun euclidean-distance-fl (p1 p2)
  "Returns a fluent-net that represents the euclidean distance between
  two poses."
  (fl-funcall #'euclidean-distance (fl-funcall #'pos p1) (fl-funcall #'pos p2)))

(defun init ()
  (roslisp:start-ros-node "turtle_cram")
  (reset-turtlesim)
  (spawn-turtle :x 1 :y 1)
  (roslisp:subscribe "/turtle1/pose" 'turtlesim-msg:<pose> (turtle-update-fun *turtle-1-pose-fluent*))
  (roslisp:subscribe "/turtle2/pose" 'turtlesim-msg:<pose> (turtle-update-fun *turtle-2-pose-fluent*))
  (roslisp:advertise "/turtle2/command_velocity" 'turtlesim-msg:<velocity>))

(defun shutdown ()
  (reset-turtlesim)
  (roslisp:shutdown-ros-node))

(def-plan go-to-point (goal &optional (distance 0.1))
  (flet ((stop ()
           (roslisp:publish "/turtle2/command_velocity" (make-instance 'turtlesim-msg:<velocity>))))
    (let ((goal-pose (pose goal)))
      (unwind-protect
           (suspend-protect
               (pursue
                 (wait-for (< (euclidean-distance-fl goal-pose *turtle-2-pose-fluent*)
                              distance))
                 (loop do
                   (roslisp:publish "/turtle2/command_velocity" (calculate-cmd goal))
                   (sleep 0.1)))
             (stop))
        ;; Always set vel to 0 when terminating
        (stop)))))

(def-plan warn-when-close (threshold)
  (whenever ((< (euclidean-distance-fl *turtle-1-pose-fluent* *turtle-2-pose-fluent*)
                threshold))
    (roslisp:ros-info :turtle "Warn: Turtles are close. Threshold: ~a~%" threshold)))

(def-plan suspend-when-close (threshold task)
  (whenever ((< (euclidean-distance-fl *turtle-1-pose-fluent* *turtle-2-pose-fluent*)
                threshold))
    (with-task-suspended task
      (wait-for (> (euclidean-distance-fl *turtle-1-pose-fluent* *turtle-2-pose-fluent*)
                   (* threshold 1.2))))))

(def-top-level-plan turtle-demo-1 ()
  (pursue
    (warn-when-close 1.4)
    (loop for p in (list (point :x 1 :y 1)
                         (point :x 9 :y 1)
                         (point :x 9 :y 9)
                         (point :x 1 :y 9)
                         (point :x 1 :y 1))
          do (go-to-point p 0.5))))

(def-top-level-plan turtle-demo-2 ()
  (pursue
    (suspend-when-close 1.4 nav-body)
    (:tag nav-body
      (loop for p in (list (point :x 1 :y 1)
                           (point :x 9 :y 1)
                           (point :x 9 :y 9)
                           (point :x 1 :y 9)
                           (point :x 1 :y 1))
            do (go-to-point p 0.5)))))

(define-condition turtle-too-close-failure (plan-error)
  ((fail-fl :initarg :fail-fl :reader fail-fl)))

(def-top-level-plan turtle-demo-3 ()
  (let ((waypoints (list (point :x 1 :y 1)
                         (point :x 9 :y 1)
                         (point :x 9 :y 9)
                         (point :x 1 :y 9)
                         (point :x 1 :y 1)))
        (close-fn (< (euclidean-distance-fl *turtle-1-pose-fluent* *turtle-2-pose-fluent*)
                     2))
        (too-close-fn (< (euclidean-distance-fl *turtle-1-pose-fluent* *turtle-2-pose-fluent*)
                         1)))
    (pursue
      (whenever (too-close-fn)
        (with-task-suspended nav-body
          (roslisp:ros-info :turtle "Stop!")
          (wait-for (not too-close-fn))
          (roslisp:ros-info :turtle "Continue!")))
      (loop do
        (loop for p in waypoints
              do
           (:tag nav-body
             (with-failure-handling
                 ((turtle-too-close-failure (f)
                    (roslisp:ros-info :turtle "Turtles too close.")
                    (pursue
                      (go-to-point (point :x 4.5 :y 4.5))
                      (wait-for (not (fail-fl f))))
                    (retry)))
               (pursue
                 (whenever (close-fn)
                   (fail 'turtle-too-close-failure :fail-fl close-fn))
                 (go-to-point p 0.2)))))))))
