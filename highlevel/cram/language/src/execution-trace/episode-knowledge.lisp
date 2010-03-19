;;;
;;; Copyright (c) 2009, Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package #:cram-execution-trace)

;;; The epsiode knowedge consists of a task tree and an execution trace. There
;;; are two kinds of episode knowledge. LIVE-EPISODE-KNOWLEDGE and
;;; OFFLINE-EPISODE-KNOWLEDGE both deriving from EPISODE-KNOWLEDGE.
;;; 
;;; LIVE-EPISODE-KNOWLEDGE supports adding new (fluent-) traces to the
;;; execution trace and its TASK-TREE references real TASK objects. It might
;;; be, that some tasks are still executed. Every TOP-LEVEL(-PLAN) is
;;; associated with its own LIVE-EPIOSDE-KNOWLEDGE, which will be reset every
;;; time the plan is executed.
;;;
;;; OFFLINE-EPISODE-KNOWLEDGE has been loaded (from a file), doesn't support
;;; additional traces being added and its TASK-TREE references OFFLINE-TASK
;;; objects which contain only the correct fluent names and result values (but
;;; e.g. no thread objects).
;;;
;;; Both kinds of EPISODE-KNOWLEDGE objects can be queried for their (current)
;;; execution-trace / task-tree and they can be saved to a file.
;;;
;;; Accessing the execution trace of a live episode is thread safe, but
;;; accessing the task tree might not be. The offline episode is not thread
;;; safe at all (but it doesn't need to be anyway).

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Interface
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpl-impl::define-task-variable *episode-knowledge* nil
  "The episode knowledge of the current episode. This is bound within a cram
   top level.")

(defvar *top-level-episode-knowledge* (make-hash-table :test 'eq)
  "The episode knowledge for top level plans is stored in this hash-table by
   the plan name (which is a symbol).")

(defun get-top-level-episode-knowledge (name)
  "Returns the episode-knowledge of the top level plan. The second return
   value is NIL if `name' does not name a top level plan."
  (declare (type symbol name))
  (gethash name *top-level-episode-knowledge*))

(defun set-top-level-episode-knowledge (name episode)
  "Set the episode-knowledge of the top level plan named by `name'."
  (declare (type symbol name))
  (setf (gethash name *top-level-episode-knowledge*)
        episode))

(defun episode-knowledge-zero-time (&optional (episode *episode-knowledge*))
  "Starting time of episode. All timestampes (e.g. in traces) can be
   interpreted relative to zero time."
  (zero-time episode))

(defun episode-knowledge-max-time (&optional (episode *episode-knowledge*))
  "Ending time of episode. This is when the last trace was recorded. Compute
   the timespan of the episode by subtracting the zero time."
  (max-time episode))

(defun episode-knowledge-task-tree (&optional (episode *episode-knowledge*))
  "The task tree for this episode."
  (task-tree episode))

(defun episode-knowledge-task-list (&optional (episode *episode-knowledge*))
  "A list of all tasks in the task tree."
  (task-list episode))

(defun episode-knowledge-goal-task-list (&optional (episode *episode-knowledge*))
  "A list of all goal tasks in the task tree."
  (goal-task-list episode))

(defun episode-knowledge-fluent-trace-queue
    (fluent-name &optional (episode *episode-knowledge*))
  "The trace queue used for the fluent named by `fluent-name'. Creates a new
   queue and adds it to the execution trace when it is not present
   allready. Works only on live episodes."
  (fluent-trace-queue episode fluent-name))

(defun episode-knowledge-traced-fluent-names (&optional (episode *episode-knowledge*))
  "Returns a list of names of all traced fluents."
  (traced-fluent-names episode))

(defun episode-knowledge-fluent-changes
    (fluent-name &optional (episode *episode-knowledge*))
  "Returns a list of cons cells - one cons for each time the fluents value
   changed - with the car being the value and cdr being the timestamp."
  (fluent-changes episode fluent-name))

(defun episode-knowledge-fluent-durations
    (fluent-name &optional (episode *episode-knowledge*))
  (fluent-durations episode fluent-name))

(defun make-episode-knowledge ()
  "Return new LIVE-EPISODE-KNOWLEDGE object."
  (make-instance 'live-episode-knowledge))

(defun reset-episode-knowledge (&optional (episode *episode-knowledge*))
  "Resets a LIVE-EPISODE-KNOWLEDGE, that is it resets its task tree and
   execution trace."
  (reset episode))

(defun save-episode-knowledge (destination
                               &optional (episode *episode-knowledge*)
                               &rest key-args)
  "Saves an episode to a file or stream. You can pass a keyword argument
   `if-exists' which is passed to OPEN if `destination' is a string or a
   pathname. Default for if-exists is :ERROR."
  (when (eq episode :if-exists)
    (push :if-exists key-args)
    (setf episode *episode-knowledge*))
  (apply #'save-episode episode destination key-args))

(defun load-episode-knowledge (source)
  "Loads an episode from a file or stream. Returns an OFFLINE-EPISODE-KNOWLDGE
   object."
  (load-episode source))

(defmacro with-episode-knowledge (episode-knowledge &body body)
  `(let ((*episode-knowledge* ,episode-knowledge))
     ,@body))

(defmacro with-top-level-episode-knowledge (plan-name &body body)
  `(with-episode-knowledge (get-top-level-episode-knowledge ',plan-name)
     ,@body))

(defmacro with-offline-episode-knowledge (source &body body)
  `(with-episode-knowledge (load-episode-knowledge ,source)
     ,@body))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Utilities
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter +equal-time-precision+ 1d-3)

(defun equal-time (t1 t2)
  (< (abs (- t1 t2)) +equal-time-precision+))

(defun count-equal-times (time insts)
  (if (or (endp insts) (not (equal-time time (timestamp (car insts)))))
      0
      (+ 1 (count-equal-times time (cdr insts)))))

(defun round-to-precision (d p)
  (declare (type double-float d)
           (type integer p))
  (let ((f (float (expt 10 p) d)))
    (/ (round (* f d)) f)))

(defun order-of-magnitude (d)
  (declare (type double-float d))
  (assert (< d 1))
  (ceiling (- (log d 10))))

(defun changes->durations (changes max-time)
  (maplist (lambda (x)
             (cons (caar x)
                   `(throughout ,(cdar x)
                                ,(if (cdr x)
                                     (cdadr x)
                                     max-time))))
           changes))

(defun calculate-max-time (execution-trace trace-list-accessor zero-time)
  (let ((max zero-time))
    (maphash-values (lambda (value)
                      (setf max (reduce #'max (funcall trace-list-accessor value)
                                        :key #'timestamp
                                        :initial-value max)))
                    execution-trace)
    ;; Make sure the max time is strictly greater than all fluent changes
    (round-to-precision (float (+ 0.1 max) max)
                        (order-of-magnitude +equal-time-precision+))))

(defun calculate-task-list (task-tree)
  (cpl:flatten-task-tree task-tree))

(defun calculate-goal-task-list (task-list)
  (remove-if-not #'cpl:goal-task-tree-node-p task-list))

(defun calculate-traced-fluent-names (execution-trace)
  (loop for k being the hash-keys in execution-trace
        collect k))

(defun calc-delta (count)
  (if (< count 10)
      (* 0.1 +equal-time-precision+)
      (* 0.1 (calc-delta (truncate count 10)))))

(defun delta-precision (d)
  (+ 1 (order-of-magnitude d)))

(defun calculate-fluent-changes (traced-instances)
  ;; We trust the fluent traces are in cronological order.
  (let ((last-time -1)
        (delta 0)
        (precision (order-of-magnitude +equal-time-precision+))
        (current 0))
    (maplist (lambda (rest)
               (let ((value (traced-value (car rest)))
                     (time (timestamp (car rest))))
                 (if (equal-time last-time time)
                     (incf current delta)
                     (let ((count (1+ (count-equal-times time (cdr rest)))))
                       (if (= 0 count)
                           (setf last-time -1
                                 delta 0
                                 precision (order-of-magnitude +equal-time-precision+)
                                 current 0)
                           (setf last-time time
                                 delta (calc-delta count)
                                 precision (delta-precision delta)
                                 current 0))))
                 (cons value (round-to-precision (+ time current) precision))))
             traced-instances)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Base class
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass episode-knowledge ()
  ((zero-time :reader zero-time :initarg :zero-time
              :type double-float)
   (task-tree :reader task-tree :initarg :task-tree
              :type cpl:task-tree-node)
   (execution-trace :initarg :execution-trace
                    :type hash-table)))

(defgeneric max-time (episode-knowledge))

(defgeneric task-list (episode-knowledge))

(defgeneric goal-task-list (episode-knowledge))

(defgeneric fluent-trace-queue (episode-knowledge fluent-name))

(defgeneric traced-fluent-instances (episode-knowledge fluent-name))

(defgeneric traced-fluents-hash-table (episode-knowledge))

(defgeneric traced-fluent-names (episode-knowledge))

(defgeneric fluent-changes (episode-knowledge fluent-name))

(defgeneric fluent-durations (episode-knowledge fluent-name))

(defgeneric reset (episode-knowledge))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Live episodes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass live-episode-knowledge (episode-knowledge)
  ((execution-trace-lock)
   ;;; maybe needed later for continuous saving of the execution-trace.
   #+nil(object-by-id :initform tg:make-weak-hash-table :weakness :value :test 'eql)
   #+nil(id-by-object :initform tg:make-weak-hash-table :weakness :key :test 'eq)))

(defmethod initialize-instance :after ((episode live-episode-knowledge) &key)
  (with-slots (execution-trace-lock execution-trace task-tree zero-time) episode
    (setf execution-trace-lock (make-recursive-lock :name "Execution trace lock.")
          execution-trace      (make-hash-table :test 'eq)
          task-tree            (cpl:make-task-tree-node)
          zero-time            (current-timestamp))))

(defmethod max-time ((episode live-episode-knowledge))
  "Compute the max time by searching all the traces."
  (with-slots (execution-trace-lock execution-trace zero-time) episode
    (with-lock-held (execution-trace-lock :whostate "max-time")
      (calculate-max-time execution-trace #'ts-queue->list zero-time))))

(defmethod task-list ((episode live-episode-knowledge))
  (calculate-task-list (task-tree episode)))

(defmethod goal-task-list ((episode live-episode-knowledge))
  (calculate-goal-task-list (task-list episode)))

(defmethod fluent-trace-queue ((episode live-episode-knowledge) fluent-name)
  (with-slots (execution-trace execution-trace-lock) episode
    (with-lock-held (execution-trace-lock :whostate "fluent-trace-queue")
      (multiple-value-bind (queue found?) (gethash fluent-name execution-trace)
        (if found?
            queue
            (setf (gethash fluent-name execution-trace)
                  (make-ts-queue (format nil "Trace of ~a" fluent-name))))))))

(defmethod traced-fluent-instances ((episode live-episode-knowledge) fluent-name)
  (with-slots (execution-trace execution-trace-lock) episode
    (with-lock-held (execution-trace-lock :whostate "traced-fluent-instances")
      (multiple-value-bind (queue found?) (gethash fluent-name execution-trace)
        (if found?
            (values (ts-queue->list queue) t)
            (values nil nil))))))

(defmethod traced-fluents-hash-table ((episode live-episode-knowledge))
  (with-slots (execution-trace execution-trace-lock) episode
    (with-lock-held (execution-trace-lock :whostate "traced-fluents-hash-table")
      (copy-hash-table execution-trace :key #'ts-queue->list))))

(defmethod traced-fluent-names ((episode live-episode-knowledge))
  (with-slots (execution-trace execution-trace-lock) episode
    (with-lock-held (execution-trace-lock :whostate "traced-fluent-names")
      (calculate-traced-fluent-names execution-trace))))

(defmethod fluent-changes ((episode live-episode-knowledge) fluent-name)
  (calculate-fluent-changes (traced-fluent-instances episode fluent-name)))

(defmethod fluent-durations ((episode live-episode-knowledge) fluent-name)
  (changes->durations (fluent-changes episode fluent-name)
                      (max-time episode)))

(defmethod reset ((episode live-episode-knowledge))
  (with-slots (execution-trace-lock execution-trace task-tree zero-time)
      episode
    (with-lock-held (execution-trace-lock :whostate "reset")
      (clrhash execution-trace))
    (cpl:clear-tasks task-tree)
    (setf zero-time (current-timestamp))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Offline episodes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass offline-episode-knowledge (episode-knowledge)
  ((max-time :reader max-time
             :type double-float)
   (task-list :reader task-list
              :type list)
   (goal-task-list :reader goal-task-list
                   :type list)
   (fluent-changes-hash-table :type hash-table)
   (fluent-durations-hash-table :type hash-table)))

(defmethod initialize-instance :after ((episode offline-episode-knowledge)
                                       &key &allow-other-keys)
  (with-slots (execution-trace max-time zero-time task-tree task-list
               goal-task-list fluent-changes-hash-table fluent-durations-hash-table)
      episode
    (setf max-time (calculate-max-time execution-trace #'identity zero-time)
          task-list (calculate-task-list task-tree)
          goal-task-list (calculate-goal-task-list task-list)
          fluent-changes-hash-table (copy-hash-table execution-trace
                                                     :key #'calculate-fluent-changes)
          fluent-durations-hash-table (copy-hash-table fluent-changes-hash-table
                                                       :key (rcurry #'changes->durations
                                                                    max-time)))))

(defmethod traced-fluent-instances ((episode offline-episode-knowledge) fluent-name)
  (with-slots (execution-trace) episode
    (gethash fluent-name execution-trace)))

(defmethod fluent-changes ((episode offline-episode-knowledge) fluent-name)
  (with-slots (fluent-changes-hash-table) episode
    (gethash fluent-name fluent-changes-hash-table)))

(defmethod fluent-durations ((episode offline-episode-knowledge) fluent-name)
  (with-slots (fluent-durations-hash-table) episode
    (gethash fluent-name fluent-durations-hash-table)))

(defmethod traced-fluents-hash-table ((episode offline-episode-knowledge))
  (slot-value episode 'execution-trace))

(defmethod traced-fluent-names ((episode offline-episode-knowledge))
  (with-slots (execution-trace) episode
    (loop for k being the hash-keys in execution-trace
       collect k)))

(defmethod fluent-trace-queue ((episode offline-episode-knowledge) fluent-name)
  (declare (ignore episode fluent-name))
  (error "Cannot trace fluents in offline episode."))

(defmethod reset ((episode offline-episode-knowledge))
  (declare (ignore episode))
  (error "Cannot reset offline episode."))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Saving / Loading
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric save-episode (episode dest &key if-exists)
  (:documentation
   "Save `episode' to `dest', so it can be restored by LOAD-EPISODE.")
  (:method (episode (dest string) &key (if-exists :error))
    (save-episode-to-file episode dest if-exists))
  (:method (episode (dest pathname) &key (if-exists :error))
    (save-episode-to-file episode dest if-exists))
  (:method (episode (dest stream) &key if-exists)
    (declare (ignore if-exists))
    (store episode dest 'episode-knowledge-backend)))

(defun save-episode-to-file (episode-knowledge dest if-exists)
  (with-backend 'episode-knowledge-backend
    (with-open-file (stream dest :direction :output
                            :element-type (stream-type *default-backend*)
                            :if-exists if-exists)
      (save-episode episode-knowledge stream))))

(defgeneric load-episode (source)
  (:documentation "Load episode knowledge that has been stored with
   SAVE-EPISODE. Returns an object of type OFFLINE-EPISODE-KNOWLEDGE.")
  (:method ((source string))
    (load-episode-from-file source))
  (:method ((source pathname))
    (load-episode-from-file source))
  (:method ((source stream))
    (restore source 'episode-knowledge-backend)))

(defun load-episode-from-file (source)
  (with-backend 'episode-knowledge-backend
    (with-open-file (stream source :direction :input
                            :element-type (stream-type *default-backend*)
                            :if-does-not-exist :error)
      (load-episode stream))))
