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

(define-cram-test wait-for-wakes-up-on-pulse
    "FIXME" ()
  (let ((fluent (make-fluent :name :test-fluent :value t)))
    (with-producer-consumer-tasks (:n-consumers 10)
      (:producer #'(lambda () (sleep 0.05) (pulse fluent)))
      (:consumer #'(lambda () (wait-for fluent))))
    (pass)))

(define-cram-test wait-for-with-timeout
    "FIXME" ()
  (let ((fluent (make-fluent :name :test-fluent :value nil)))
    (wait-for fluent :timeout 0.05)
    (pass)))

(define-cram-test wait-for-fluent-network-not-eq
    "FIXME" ()
  (let* ((fluent (make-fluent :name :test-fluent :value :wait-for-me))
         (fluent-net (not (eq fluent :wait-for-me))))
    (with-producer-consumer-tasks (:n-consumers 10)
      (:producer #'(lambda ()
                     (sleep 0.05)
                     (setf (value fluent) :stop-waiting)))      
      (:consumer #'(lambda ()
                     (wait-for fluent-net))))
    (pass)))

(define-cram-test wait-for-fluent-network-fl-funcall
    "FIXME"
    ((:generators (a-value (gen-one-element :a :b :c :d))))
  (let* ((fluent (make-fluent :name :test-fluent :value :wait-for-me))
         (fluent-net (fl-funcall #'member fluent '(:a :b :c :d))))
    (with-producer-consumer-tasks (:n-consumers 10)
      (:producer #'(lambda ()
                     (sleep 0.05)
                     (setf (value fluent) a-value)))
      (:consumer #'(lambda ()
                     (wait-for fluent-net))))
    (pass)))

(define-cram-test whenever-always-triggered
    "FIXME"
    ((:timeout 10.0)
     (:generators (n-consumers (gen-integer :min 1 :max 25))
                  (n-triggers  (gen-integer :min 1 :max 50))))
  (let ((fluent (make-fluent :name :test-fluent :value t)))
    (with-producer-consumer-tasks (:n-producers 1
                                   :n-consumers n-consumers)
      (:producer #'(lambda ()
                     ;; Sleeps are evil, but we need to let the
                     ;; consumers enter the whenever state. If we are
                     ;; finished producing before all consumers
                     ;; entered whenever, some block since all pulses
                     ;; are done already.
                     (sleep 0.5)
                     (loop repeat n-triggers
                           do (pulse fluent))))
      (:consumer #'(lambda ()
                     (let ((i 1))
                       (whenever (fluent :handle-missed-pulses :always)
                         (when (= (incf i) n-triggers)
                           (return-from whenever)))))))
    (pass)))

(define-cram-test whenever-once-triggered
    "FIXME"
    ((:timeout 5.0)
     (:generators (n-consumers (gen-integer :min 1 :max 50))
                  (n-producers (gen-integer :min 1 :max 50))
                  (n-triggers  (gen-integer :min 1 :max 100))))
  (let ((fluent (make-fluent :name :test-fluent :value 0)))
    (with-producer-consumer-tasks (:n-producers n-producers
                                   :n-consumers n-consumers)
      (:producer #'(lambda ()
                     (loop for i from 1 to n-triggers
                           do (setf (value fluent) i))))
      (:consumer #'(lambda ()
                     (whenever ((fl-pulsed fluent) :handle-missed-pulses :once)
                       (when (eql (value fluent) n-triggers)
                         (return-from whenever))))))
    (pass)))

(define-cram-test fluent-net-many-inputs
    "FIXME"
     ((:timeout 2.5))
  (let* ((fl-1 (make-fluent :name :fl-1 :value 0))
         (fl-2 (make-fluent :name :fl-2 :value 0))
         (fl-net (fl+ fl-1 fl-2)))
    (declare (ignore fl-net))
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
              (incf (value fl-2)))))
    (pass)))