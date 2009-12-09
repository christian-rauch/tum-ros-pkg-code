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

(def-suite language-base)

(in-suite language-base)

(test top-level-terminates
  (for-all ()
    (let ((top-level-finished nil))
      (with-deadlock-handling (:timeout 0.1)
          (top-level
            (setf top-level-finished t))
        (is (not (null top-level-finished)))))))

(test test-par-terminates
  (for-all ()
    (let ((finished (make-array 2 :initial-element nil)))
      (with-deadlock-handling (:timeout 0.1)
          (top-level
            (par
              (setf (elt finished 0) t)
              (setf (elt finished 1) t)))
        (is (every #'identity finished))))))

(test test-par-performs-parallel
  (for-all ()
    (let ((ping-pong (make-fluent :name :ping-pong :value nil)))
      (with-deadlock-handling (:timeout 0.15)
          (top-level
            (par
              (seq
                (setf (value ping-pong) :ping)
                (wait-for (eq ping-pong :pong)))
              (seq
                (wait-for (eq ping-pong :ping))
                (setf (value ping-pong) :pong))))))))

(test test-par-failure-handling
  (for-all ()
    (let ((failure nil)
          (worker-terminated nil))
      (with-deadlock-handling (:timeout 0.15)
          (handler-case
              (top-level
                (par
                  (seq
                    (sleep 0.01)
                    (fail))
                  (seq
                    (sleep 1)
                    (setf worker-terminated t))))
            (error (e)
              (setf failure e)))
        (is (not worker-terminated))
        (is (typep failure 'plan-error))))))

;; (test test-with-tags
;;   (for-all ()
;;     (let ((finished (make-array 2 :initial-element nil)))
;;       (with-deadlock-handling (:timeout 0.15)
;;           (top-level
;;             (with-tags
;;               (:tag task-1
;;                     (setf (aref finished 0) t))
;;               (:tag task-2
;;                     (setf (aref finished 1) t))))
;;         (is (every #'identity finished))))))

(test test-par-failure-final-status-of-tasks
  (for-all ()
    (let ((producer-thread nil)
          (worker-1-thread nil)
          (worker-2-thread nil)
          (worker-3-thread nil))
      (with-deadlock-handling (:timeout 1.00)
          (top-level
            (with-tags
              (ignore-some-conditions (cpl:simple-plan-error cpl-impl::rethrown-error)
                (par
                  (:tag producer
                    (sleep 0.01)
                    (fail))
                  (:tag worker-1
                    (par
                      (:tag worker-2
                        (sleep 10))
                      (:tag worker-3
                        (sleep 10))))))
              (setf producer-thread producer)
              (setf worker-1-thread worker-1)
              (setf worker-2-thread worker-2)
              (setf worker-3-thread worker-3))))
      (is (eq (value (status producer-thread)) :failed) (format nil "~a not failed" producer-thread))
      (is (eq (value (status worker-1-thread)) :evaporated))
      (is (eq (value (status worker-2-thread)) :evaporated))
      (is (eq (value (status worker-3-thread)) :evaporated)))))

(test test-task-blocking
  (flet ((time-equal (t1 t2  &key (threshold 0.01))
           (<  (/ (abs (- t2 t1))
                  internal-time-units-per-second)
               threshold)))
    (for-all ()
      (let ((final-times (make-array 2))
            (blockee-trigger (make-fluent :name :blockee-trigger :value nil)))
        (with-deadlock-handling (:timeout 0.5)
            (top-level
              (with-tags
                (par
                  (:tag blocker
                    (let ((start-time nil))
                      (wait-for (eq (status blockee) :waiting))
                      (with-task-suspended blockee
                        (setf start-time (get-internal-real-time))
                        (setf (value blockee-trigger) t)
                        (sleep 0.1)
                        (setf (aref final-times 0) (- (get-internal-real-time) start-time)))))
                  (:tag blockee
                    (let ((start-time nil))
                      (on-suspension
                          (setf start-time (get-internal-real-time))
                        (wait-for blockee-trigger))
                      (setf (aref final-times 1) (- (get-internal-real-time) start-time)))))))
          (is (>= (aref final-times 1) (aref final-times 0))))))))

;;; Deprecated! We now have new semantics for suspend-protect
;; (test test-suspend-protect
;;   (flet ((time-equal (t1 t2  &key (threshold 0.01))
;;            (<=  (/ (abs (- t2 t1))
;;                    internal-time-units-per-second)
;;                 threshold)))
;;     (for-all ()
;;       (let ((protected nil)
;;             (runs 0))
;;       (with-deadlock-handling (:timeout 0.30)
;;             (top-level
;;               (with-tags
;;                 (par
;;                   (:tag blocker
;;                     (wait-for (not (eq (status blockee) :created)))
;;                     (sleep 0.05)
;;                     (with-task-suspended blockee
;;                       (sleep 0.05)))
;;                   (:tag blockee
;;                     (suspend-protect
;;                         (progn
;;                           (incf runs)
;;                           (sleep 0.15))
;;                       (setf protected t))))))
;;           (is (eq protected t))
;;           (is (eql runs 2)))))))

(test test-pursue
  (for-all ()
    (let ((worker-1-terminated nil)
          (worker-2-terminated nil))
      (with-deadlock-handling (:timeout 0.25)
          (top-level
            (with-tags
              (pursue
                (:tag worker-1
                  (sleep 0.2)
                  (setf worker-1-terminated t))
                (:tag worker-2
                  (sleep 0.05)
                  (setf worker-2-terminated t)))))
        (is (not worker-1-terminated))
        (is (not (null worker-2-terminated)))))))

(test test-try-all
  (for-all ()
    (let ((worker-1-finished nil)
          (worker-2-finished nil))
      (with-deadlock-handling (:timeout 0.25)
          (top-level
            (try-all
              (setf worker-1-finished t)
              (seq
                (sleep 0.15)
                (setf worker-2-finished t))))
        (is (not (null worker-1-finished))
            (not worker-2-finished))))))

(test test-try-all-failure
  (for-all ()
    (let ((worker-1-exec nil)
          (worker-2-exec nil)
          (worker-3-exec nil)
          (err nil))
      (with-deadlock-handling (:timeout 0.5)
          (with-failure-handling
              ((plan-error (e)
                 (setf err e)
                 (return nil)))
            (top-level
              (try-all
                (and (setf worker-1-exec t) (fail))
                (and (setf worker-2-exec t) (fail))
                (and (setf worker-3-exec t) (fail)))))
        (is (not (null worker-1-exec)))
        (is (not (null worker-2-exec)))
        (is (not (null worker-3-exec)))
        (is (typep err 'composite-failure))))))

(test test-try-in-order
  (for-all ()
    (let ((worker-1-exec nil)
          (worker-2-exec nil))
      (with-deadlock-handling (:timeout 0.1)
          (top-level
            (try-in-order
              (seq
                (setf worker-1-exec t)
                (fail))
              (setf worker-2-exec t)))
        (is (not (null worker-1-exec)))
        (is (not (null worker-2-exec)))))))

(test test-try-in-order-failure
  (for-all ()
    (let ((worker-1-exec nil)
          (worker-2-exec nil)
          (worker-3-exec nil)
          (err nil))
      (with-deadlock-handling (:timeout 0.1)
          (handler-case (top-level
                          (try-in-order
                            (and (setf worker-1-exec t) (fail))
                            (and (setf worker-2-exec t) (fail))
                            (and (setf worker-3-exec t) (fail))))
            (plan-error (e)
              (setf err e)))
        (is (not (null worker-1-exec)))
        (is (not (null worker-2-exec)))
        (is (not (null worker-3-exec)))
        (is (typep err 'composite-failure))))))
