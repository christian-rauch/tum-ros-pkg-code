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

(defclass barrier ()
  ((value :initform (error "Value must be specified")
          :initarg :value)
   lock waitqueue))

(defgeneric barrier-wait (barrier &key timeout))

(defmethod initialize-instance :after ((barrier barrier) &key &allow-other-keys)
  (with-slots (lock waitqueue) barrier
    (setf lock (make-lock))
    (setf waitqueue (make-condition-variable :lock lock))))

(defmethod barrier-wait ((barrier barrier) &key (timeout nil))
  (with-slots (value lock waitqueue) barrier
    (with-lock-held (lock)
      (cond ((> value 0)
             (decf value)
             (if timeout
                 (condition-variable-wait-with-timeout waitqueue timeout)
                 (condition-variable-wait waitqueue)))
            (t
             (condition-variable-broadcast waitqueue))))))

(defun wait-for-thread-count (count timeout &optional (partitions 100))
  (loop repeat partitions
     with timeout-part = (/ timeout partitions)
     until (eql count (length (all-threads)))
     do (sleep timeout-part)))

(defmacro with-producer-consumer-threads ((&key (num-consumers 1) (timeout 0.02))
                                          (&body producer)
                                          (&body consumer)
                                          &body tests)
  `(flet ((producer () ,@producer)
          (consumer () ,@consumer))
     (let* ((thread-count (length (all-threads)))
            (producer-running nil)
            (producer-running-lock (make-lock))
            (producer-running-barrier (make-condition-variable
                                       :lock producer-running-lock))
            (finished-barrier (make-instance 'barrier :value (+ ,num-consumers 1)))
            (producer-thread (spawn-thread "Producer"
                                           (lambda ()
                                             (with-lock-held (producer-running-lock)
                                               (setf producer-running t)
                                               (condition-variable-broadcast producer-running-barrier))
                                             (producer)
                                             (barrier-wait finished-barrier))))
            (consumer-threads (loop for i from 1 to ,num-consumers
                                 collect (spawn-thread "Consumer"
                                                       (lambda ()
                                                         (with-lock-held (producer-running-lock)
                                                           (unless producer-running
                                                             (condition-variable-wait producer-running-barrier)))
                                                         (consumer)
                                                         (barrier-wait finished-barrier))))))
       (unwind-protect
            (progn
              (barrier-wait finished-barrier :timeout ,timeout)
              (wait-for-thread-count thread-count ,timeout)
              (is (not (thread-alive-p producer-thread)) "Producer thread termination.")
              (is-false (some #'thread-alive-p consumer-threads)
                        (format nil "Consumer threads termination. ~a threads running."
                                (reduce (lambda (sum current)
                                          (if (thread-alive-p current)
                                              (1+ sum)
                                              sum))
                                        consumer-threads :initial-value 0))))
         (when (thread-alive-p producer-thread)
           (kill-thread producer-thread))
         (loop for thread in consumer-threads
            when (thread-alive-p thread) do (kill-thread thread)))
       ,@tests)))

(defmacro with-deadlock-handling ((&key (timeout 0.2)) thread-body
                                  &body test-body)
  `(flet ((thread-fun ()
            ,thread-body))
     (let* ((number-of-threads (length (all-threads)))
            (barrier (make-instance 'barrier :value 1))
            (thread (spawn-thread "test" (lambda ()
                                           (thread-fun)
                                           (barrier-wait barrier)))))
       (barrier-wait barrier :timeout ,timeout)
       ;; Ugly, but we need to sleep some time to let thread terminations
       ;; settle down when some threads are still running.
       (wait-for-thread-count number-of-threads ,timeout)
       (unwind-protect
         (is (not (thread-alive-p thread)) "Thread deadlock.")
         (when (thread-alive-p thread)
           (kill-thread thread)))
       (is (eql number-of-threads (length (all-threads))) "Threads left back.")
       ,@test-body)))
