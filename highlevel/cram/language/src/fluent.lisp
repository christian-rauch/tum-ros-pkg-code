;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;;                     Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package :cpl-impl)

(defclass fluent ()
  ((value :initarg :value :initform nil
          :documentation "The value of the fluent")
   (pulse-count :initform 0 :documentation "For internal use. Indicates a pulse.")
   (subscribed-threads :initform (list)
                       :documentation "alist of subscribed threads and the pulse-count
                                       of the last pulse received by the thread.")
   (name :initarg :name :initform (error "No name specified.")
         :reader name :type symbol
         :documentation "The name of the fluent. Should be a globally unique symbol.")
   (test :initarg :test :initform #'eql
         :documentation "Test to check for value changes (used in wait-for).")
   (on-update :initform (make-hash-table :test 'cl:eq) :type hash-table
              :documentation "Hash-table of update callbacks, being executed on every change of
                              the value. The update callback is a function with no parameters.")
   (changed-condition :documentation "For internal use. Posix condition variable/waitqueue
                                      used for notification when value changed.")
   (value-lock :documentation "For internal use. Lock to be used with for access synchronization.")
   (dependencies :initarg :dependencies
                 :documentation "For internal use. Contains the list
                                 of fluents this fluent depends on.")))

(defgeneric value (fluent)
  (:documentation "Reader method, returning the fluent's value"))

(defgeneric (setf value) (new-value fluent)
  (:documentation "Setter method to set the new value of the fluent"))

(defgeneric register-update-callback (fluent name update-fun)
  (:documentation "Method to register an update callback under the corresponding name.
                   When the name is already known, an error is signaled."))

(defgeneric remove-update-callback (fluent name)
  (:documentation "Method to remove the update callback with the given name."))

(defgeneric wait-for (fluent &key timeout value-changed wait-status handle-missed-pulses)
  (:documentation "Method to block the current thread until the value
                   of the fluent becomes non-nil. When it is already
                   true and value-changed is nil, wait-for imediately
                   returns. Otherwise it waits for at least timeout.
                   The parameter wait-status indicates the status of
                   the thread when it is waiting. The parameter
                   handle-missed-pulses can be either :never :once
                   or :always. :never means that missed pulses are
                   ignored, :once means that independent of the number
                   of missed pulses, they are handled only once
                   and :always means that the internal pulse count is
                   only incremented by one, allowing for being
                   triggered on every missed pulse. The handling of
                   missed pulses is performed on a per-thread
                   basis."))

(defgeneric pulse (fluent)
  (:documentation "Method to trigger the fluent, i.e. notifying all waiting threads,
                   but without actually changing the fluent value."))

(defgeneric subscribe-current-thread (fluent)
  (:documentation "For internal use only: Subscribes the current
  thread to perform wait-fors. This method registers the thread with
  the current pulse-count. wait-for can then figure out if there are
  pending pulses not yet processed by the corresponding thread."))

(defgeneric on-update-hook (fluent)
  (:documentation "For internal use only: Is called everytime a pulse happens,
  after pulse-count is incremented, but before changed-condition is signaled
  and the on-update callbacks are executed. The call happens while holding the
  lock for the fluent value (to make sure hooks are called in the same order
  as pulses are). ON-UPDATE-HOOK is intended to be used for tracing fluent
  value changes."))

(defmethod value (var)
  "Default handler. It seems to be a quite good idea to allow the
  usage of value for all types of objects."
  var)

(defmethod wait-for (var &key &allow-other-keys)
  (or var (error "wait-for called with nil argument.")))

(defmethod initialize-instance :after ((fluent fluent) &key)
  (with-slots (name changed-condition value-lock) fluent
    (setf value-lock (make-recursive-lock :name (format nil "~a-lock" name)))
    (setf changed-condition (make-condition-variable :lock value-lock)))
  (subscribe-current-thread fluent))

(defmethod print-object ((fluent fluent) stream)
  (print-unreadable-object (fluent stream :type t :identity t)
    (format stream "[~s]" (value fluent))))

(defmethod value ((fluent fluent))
  (with-unsuspendable-lock (slot-value fluent 'value-lock)
    (slot-value fluent 'value)))

(defmethod (setf value) (new-value (fluent fluent))
  (with-slots (value-lock test value) fluent
    (without-termination
      (with-unsuspendable-lock value-lock
        (unless (funcall test value new-value)
          (setf (slot-value fluent 'value) new-value)
          (pulse fluent)))))
  new-value)

(defmethod register-update-callback ((fluent fluent) name update-fun)
  (with-unsuspendable-lock (slot-value fluent 'value-lock)
    (setf (gethash name (slot-value fluent 'on-update)) update-fun)))

(defmethod remove-update-callback ((fluent fluent) name)
  (with-unsuspendable-lock (slot-value fluent 'value-lock)
    (remhash name (slot-value fluent 'on-update))))

(defmethod subscribe-current-thread ((fluent fluent))
  (with-slots (subscribed-threads pulse-count value-lock) fluent
    (flet ((gc-subscriptions ()
             (loop for subscription in subscribed-threads
                for (thread . count) = subscription
                when (tg:weak-pointer-value thread) collect subscription into result
                finally (setf subscribed-threads result))))
      (let ((weak-thread (tg:make-weak-pointer (current-thread))))
        (with-unsuspendable-lock value-lock
          (or (assoc (current-thread) subscribed-threads :key #'tg:weak-pointer-value)
              (let ((new-entry (cons weak-thread pulse-count)))
                (gc-subscriptions)
                (setf subscribed-threads
                      (push new-entry subscribed-threads))
                new-entry)))))))

(defmethod wait-for ((fluent fluent) &key
                     (timeout nil) (value-changed nil)
                     (wait-status :waiting) (handle-missed-pulses :once))
  (with-slots (changed-condition value-lock value pulse-count) fluent
    (with-status (wait-status)
      (let ((subscription-info (subscribe-current-thread fluent))
            (start-time (get-internal-real-time)))
        (destructuring-bind (thread . old-pulse-count) subscription-info
          (declare (ignore thread))
          (without-suspension
            (or (with-lock-held (value-lock)
                  (when (> pulse-count old-pulse-count)
                    (ecase handle-missed-pulses
                      (:never (setf (cdr subscription-info) pulse-count)
                              nil)
                      (:once (setf (cdr subscription-info) pulse-count))
                      (:always (incf (cdr subscription-info))))))
                (unless value-changed
                  (with-lock-held (value-lock)
                    value))
                ;; We need to do ugly explicit handling of suspension
                ;; here.  WAIT-FOR is a very special case since we need
                ;; to assure that even if CONDITION-VARIABLE-WAIT gets
                ;; suspended, the value-lock is released and we wait
                ;; again after suspension.
                (let ((old-pulse-count (with-lock-held (value-lock)
                                         pulse-count)))
                  (tagbody retry
                     (with-suspension
                       (handler-case
                           (with-lock-held (value-lock)
                             (unless (value (> pulse-count old-pulse-count))
                               (if timeout
                                   (condition-variable-wait-with-timeout changed-condition timeout)
                                   (condition-variable-wait changed-condition))))
                         (suspension (c)
                           (signal-suspension c)
                           t)))
                     (unless (or (and timeout (>= (/ (- (get-internal-real-time)
                                                        start-time)
                                                     internal-time-units-per-second)
                                                  timeout))
                                 (with-lock-held (value-lock)
                                   (and value
                                        (> pulse-count old-pulse-count))))
                       (go retry))))))
          value)))))

(defmethod pulse ((fluent fluent))
  (with-slots (changed-condition pulse-count on-update value-lock) fluent
    (without-termination
      (with-unsuspendable-lock value-lock
        (incf pulse-count)
        (on-update-hook fluent)
        (condition-variable-broadcast changed-condition)
        (loop with on-update-copy = (copy-hash-table on-update)
           for callback being the hash-values of on-update-copy
           do (funcall callback))))))


(defmethod on-update-hook ((fluent fluent))
  "Default hook for normal fluents does nothing.")

(defmacro whenever ((condition-fl &key (wait-status :waiting) (handle-missed-pulses :once))
                    &body body)
  "Executes body whenever condition-fluent is pulsed or non-nil.
   The parameters wait-status and handle-missed-pulses have the same meaning as in wait-for"
  `(flet ((loop-body ()
             ,@body))
     (let ((condition-fl-var ,condition-fl))
       (with-slots (value value-lock) condition-fl-var
         (loop
            when (with-unsuspendable-lock value-lock value) do (loop-body)
            do (wait-for condition-fl-var
                         :value-changed t
                         :wait-status ,wait-status
                         :handle-missed-pulses ,handle-missed-pulses))))))

(defmacro with-fluent-locked (fluent &body body)
  `(with-unsuspendable-lock (slot-value ,fluent 'value-lock)
     ,@body))
