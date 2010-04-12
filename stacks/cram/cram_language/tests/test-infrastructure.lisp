;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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


(in-package :cpl-tests)

(defmacro with-producer-consumer-threads
    ((&key (n-producers 1) (n-consumers 1)) &body ((&key producer)
                                                   (&key consumer)))
  (assert producer) (assert consumer)
  `(let ((producer-thunk ,producer)
         (consumer-thunk ,consumer)
         (n-threads (+ ,n-producers ,n-consumers)))
     (with-synchronization-barrier (all-ready n-threads)
       (let ((producers (spawn-threads ,n-producers "Producer"
                                       (lambda ()
                                         (enter-barrier all-ready)
                                         (funcall producer-thunk))))
             (consumers (spawn-threads ,n-consumers "Consumer"
                                       (lambda ()
                                         (enter-barrier all-ready)
                                         (funcall consumer-thunk)))))
         (mapc #'join-thread producers)
         (mapc #'join-thread consumers)))))


(defvar *spawned-threads*)
(declaim (type queue *spawned-threads*))

(defun make-thread-and-enqueue (function &key name)
  (assert (boundp '*spawned-threads*) ()
          "Must be used within the extent of DEFINE-CRAM-TEST.")
  ;; Disable interrupts so we won't leave orphans behind...
  (sb-sys:without-interrupts
    (let ((thread (sb-thread:make-thread function :name name)))
      (enqueue thread *spawned-threads*)
      thread)))

(defun list-spawned-threads ()
  (list-queue-contents *spawned-threads*))

(defun assert-spawned-threads-dead ()
  (let ((alive (remove-if-not #'thread-alive-p (list-spawned-threads))))
    (assert (null alive) ()
            "~@<Test returned success, yet the following threads ~
                were still alive: ~2I~:_~{~A~^, ~}.~@:>"
            (mapcar #'thread-name alive))))

(defun shutdown-spawned-threads (n-repeats)
  (let ((already-dead    (cons 0 nil))
        (already-aborted (cons 0 nil))
        (killed          (cons 0 nil))
        (unkillable      (cons 0 nil)))
    (flet* ((add (thread box)
             (incf (car box))
             (push (thread-name thread) (cdr box)))
            (shutdown () 
             (loop for thread = (dequeue *spawned-threads*)
                   while thread do
                   (cond ((not (thread-alive-p thread))
                          (if (eq (join-thread thread) :error)
                              (add thread already-aborted)
                              (add thread already-dead)))
                         ((kill-thread thread)
                          (add thread killed))
                         (t
                          (case (join-thread thread :timeout 0.1)
                            ((:error)   (add thread already-aborted))
                            ((:timeout) (add thread unkillable))
                            (t          (add thread already-dead))))))))
      ;; There's a race at play here: killing a thread may have some
      ;; small delay, and during that delay, it may spawn new
      ;; threads. That may happen exactly after DEQUEUE returned NIL.
      ;; We kill N-REPEATS times so the likelyhood of the race is kept
      ;; very very small.
      (loop repeat n-repeats do (shutdown) (sleep 0.05))
      (values (reduce #'+ (list already-dead already-aborted
                                killed unkillable)
                      :key #'car)
              killed unkillable
              already-dead already-aborted))))

(defun serious-test-failure (condition)
  (multiple-value-bind (total killed unkillable already-dead already-aborted)
      (shutdown-spawned-threads 3)
    (symbol-macrolet
        ((n-killed                (car killed))
         (n-unkillable            (car unkillable))
         (n-already-dead          (car already-dead))
         (n-already-aborted       (car already-aborted))
         (killed-threads          (cdr killed))
         (unkillable-threads      (cdr unkillable))
         (already-dead-threads    (cdr already-dead))
         (already-aborted-threads (cdr already-aborted)))
      ;; The per-line-prefix spaces are chosen so they fit well with how
      ;; 5am reports failures.
      (5am:fail
       "~%~
        ~@<    ~@;~
          Preempted due to ~S:~@:_~
              ~<    ~@;\"~A\"~:>~@:_~@:_~
          Summary:~1I~@:_~
            ~3D threads total~
            ~:[~@:_~3D terminated before preemption: ~<~@{~A~^, ~:_~}~:>~;~2*~]~
            ~:[~@:_~3D aborted before preemption:    ~<~@{~A~^, ~:_~}~:>~;~2*~]~
            ~:[~@:_~3D killed after preemption:      ~<~@{~A~^, ~:_~}~:>~;~2*~]~
            ~:[~@:_~3D unkillable ghosts:            ~<~@{~A~^, ~:_~}~:>~;~2*~]~
        ~:>"
       condition (list condition)
       total
       (zerop n-already-dead)    n-already-dead     already-dead-threads
       (zerop n-already-aborted) n-already-aborted  already-aborted-threads
       (zerop n-killed)          n-killed           killed-threads
       (zerop n-unkillable)      n-unkillable       unkillable-threads))))

(defun run-cram-test (timeout thunk)
  ;; N.b. unwinding by HANDLER-CASE out of the WITHOUT-TIMEOUT is
  ;; important so we won't get caught by a timeout during execution of
  ;; the handler.
  (handler-case
      ;; We try to enforce the timeout with a deadline first because
      ;; unwinding from such a place is deemed to be safer.
      (multiple-value-prog1 (sb-ext:with-timeout (+ timeout 0.01)
                              (sb-sys:with-deadline (:seconds timeout)
                                (funcall thunk)))
        (assert-spawned-threads-dead))
    (serious-condition (c)
      (serious-test-failure c)
      c)))

(defparameter +timeout+ 1.0
  "Default timeout per run if not explicitly given.")

(defparameter +n-runs+ 10
  "Default number of runs a test should be repeated.")

(defparameter *compile-tests-at* :definition-time
  "We want the tests to be compiled at definition time so SBCL will
  emit warnings at compilation time to catch typos, etc. However,
  compiling them at definition time, means that tests have to be
  recompiled in case macros are changed.")

(declaim (type (member :definition-time :run-time) *compile-tests-at*))

(defmacro define-cram-test (name clauses docstring &body body)
  "Some sugar on top of 5AM:TEST.

During the execution of BODY

  - errors are caught and turned into failures.

  - hangs are caught by an implicitly, or explicitly specified
    timeout.

  - spawned threads are collected. In case of an error, or timeout,
    these threads are killed. (They're not automatically killed in
    case of success, or 5AM:IS failure.)

  - in case of an error, or timeout, some statistics is printed along
    the error message on how many threads were created, how many
    threads terminated by themselves, how many threads aborted due to
    an error by themselves, and how many threads had to be killed.

CLAUSES can be one of

  (:N-RUNS <n>)

    specifies how many times BODY is executed. Defaults to
    5AM::*NUM-TRIALS* which again defaults to 100.

  (:TIMEOUT <timeout>)

    specifies a timeout in seconds, _per run_. If BODY does not
    terminate until the timeout expired, a failure is generated.
    Defaults to +TIMEOUT+.

  (:GENERATORS <bindings>+)

    specifies bindings for 5AM's random testing facility as per
    5AM:FOR-ALL.

A documentation string must be given. Please document what your test
cases are supposed to test.
"
  (check-type docstring string)
  (labels ((genname (thing)
             (format nil "~A for test ~S" thing name))
           (default (x default-value)
             (if (eq x :default) default-value x))
           (assert-default (tag var)
             (assert (eq var :default) () "~S specified twice." tag))
           (parse-clauses (clauses)
             (let ((timeout    :default)
                   (generators :default)
                   (n-runs     :default))
               (dolist (clause clauses (values
                                        (default timeout +timeout+)
                                        (default generators nil)
                                        (default n-runs +n-runs+)))
                 (destructure-case clause
                   ((:timeout x)
                    (assert-default :timeout timeout)
                    (setq timeout x))
                   ((:generators . gs)
                    (assert-default :generators generators)
                    (setq generators gs))
                   ((:n-runs n)
                    (assert-default :n-runs n-runs)
                    (setq n-runs n)))))))
    (multiple-value-bind (timeout generators n-runs)
        (parse-clauses clauses)
      ;; We want the tests to be compiled at definition time so we
      ;; will get warnings etc. at compile-time; of course, that won't
      ;; take macro
      `(5am:test (,name :compile-at ,*compile-tests-at*)
         (let ((5am::*num-trials* ,n-runs)
               (*make-thread-function* #'make-thread-and-enqueue)
               (*spawned-threads*
                (make-queue :name ,(genname "*SPAWNED-THREADS*"))))
           (format 5am:*test-dribble* "~&Running ~S (~Dx): " ',name ,n-runs)
           (5am:for-all ,generators
             (run-cram-test ,timeout #'(lambda () ,@body))))))))
