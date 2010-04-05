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

(def-suite fluent-tests :in language)

(in-suite fluent-tests)

(test wait-for-wakes-up-on-pulse
  (for-all ()
    (let ((fluent (make-fluent :name :test-fluent :value t)))
      (with-producer-consumer-threads (:num-consumers 10 :timeout 0.2)
        ((sleep 0.05)
         (pulse fluent))
        ((wait-for fluent))))))

(test wait-for-with-timeout
 (for-all ()
   (let ((fluent (make-fluent :name :test-fluent :value nil)))
     (with-deadlock-handling (:timeout 0.2)
         (wait-for fluent :timeout 0.05)))))

(test wait-for-fluent-network-not-eq
  (for-all ()
    (let* ((fluent (make-fluent :name :test-fluent :value :wait-for-me))
           (fluent-net (not (eq fluent :wait-for-me))))
      (with-producer-consumer-threads (:num-consumers 10 :timeout 0.2)
        ((sleep 0.05)
         (setf (value fluent) :stop-waiting))
        ((wait-for fluent-net))))))

(test wait-for-fluent-network-fl-funcall
  (let ((possible-values '(:a :b :c :d)))
    (for-all ((one-value (apply #'gen-one-element possible-values)))
      (let* ((fluent (make-fluent :name :test-fluent :value :wait-for-me))
             (fluent-net (fl-funcall #'member fluent possible-values)))
        (with-producer-consumer-threads (:num-consumers 10 :timeout 0.2)
          ((sleep 0.05)
           (setf (value fluent) one-value))
          ((wait-for fluent-net)))))))

(test whenever-always-triggered
  (let ((num-triggers 10)
        (num-wakeups 0))
    (for-all ()
      (let ((fluent (make-fluent :name :test-fluent :value t)))
        (with-producer-consumer-threads (:num-consumers 10 :timeout 5)
          ((sleep 0.1)
           (loop for i from 1 to num-triggers
                do (pulse fluent)))
          ((block nil
             (let ((iterations 1))
               (whenever (fluent :handle-missed-pulses :always)
                 (incf iterations)
                 (incf num-wakeups)
                 (when (eq iterations num-triggers)
                   (return)))))))))))

(test whenever-once-triggered
  (for-all ()
    (let ((fluent (make-fluent :name :test-fluent :value 0))
          (num-triggers 10)
          (num-wakeups 0))
      (with-producer-consumer-threads (:num-consumers 10 :timeout 0.2)
        ((sleep 0.05)
         (loop for i from 1 to num-triggers
            do (setf (value fluent) i)))
        ((block nil
           (whenever ((pulsed fluent) :handle-missed-pulses :once)
             (incf num-wakeups)
             (when (eql (value fluent) num-triggers)
               (return)))))
        (is (> num-wakeups 0))))))

(test fluent-net-many-inputs
  (for-all ()
    (let* ((fl-1 (make-fluent :name :fl-1 :value 0))
           (fl-2 (make-fluent :name :fl-2 :value 0))
           (fl-net (+ fl-1 fl-2)))
      (with-deadlock-handling (:timeout 2.5)
          (top-level
            (pursue
              ;; (whenever (fl-net :handle-missed-pulses :always)
              ;;   (incf trigger-count)
              ;;   (format t "ping ~a~%" trigger-count))
              (loop repeat 100
                    do
                 (incf (value fl-1)))
              (loop repeat 100
                    do
                 (incf (value fl-2)))))))))