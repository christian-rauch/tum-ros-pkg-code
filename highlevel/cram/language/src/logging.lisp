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

(in-package :cpl)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Logging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *instance-log* (make-ts-queue "Global Instance Log")
  "Changes in logged fluents are put on a thread safe queue")

(defun clear-instance-log ()
  (ts-queue-clear *instance-log*))

(defun get-logged-instances ()
  (ts-queue->list *instance-log*))

(defun get-logged-timespan ()
  (let ((l (sort (get-logged-instances) #'< :key #'timestamp)))
    (values (timestamp (first l))
            (timestamp (car (last l))))))

(defun get-max-logged-time ()
  (multiple-value-bind (min max) (get-logged-timespan)
    (declare (ignore min))
    max))

(defun get-min-logged-time ()
  (multiple-value-bind (min max) (cpl::get-logged-timespan)
    (declare (ignore max))
    min))

(defclass logged-instance ()
  ((id :initform nil :initarg :id :reader id)
   (timestamp :initform nil :initarg :timestamp :reader timestamp))
  (:documentation "Logged instances are the objects actually saved in the log."))

(defmethod initialize-instance :after ((instance logged-instance) &key &allow-other-keys)
  (with-slots (id timestamp) instance
    (setf id        (or id        (get-eq-id instance))
          timestamp (or timestamp (current-timestamp)))))

(defgeneric append-to-log (obj)
  (:documentation "Appends a logged instance to the according log.
                   Define new methods, if you want to use specific logs
                   (queues, files, etc) for specific types.")
  (:method ((obj logged-instance))
    (ts-enqueue *instance-log* obj)))

(defclass logging-instance-mixin ()
  ()
  (:documentation "Use this mixin for classes you want to log."))

;;; Always log upon creation
(defmethod initialize-instance :after ((instance logging-instance-mixin) &key &allow-other-keys)
  (log-instance instance))

(defgeneric make-logged-instance (obj)
  (:documentation "This should return an object of type 'logged-instance (or subclass).
                   It should be a snapshot of the current state of obj to be logged.
                   You will want to specialize this for subclasses of loggin-instance-mixin."))

(defgeneric log-instance (obj)
  (:documentation "Uses make-logged-instance to create a log and appends it to the log.
                   You usually don't need to specialize this.")
  (:method ((obj logging-instance-mixin))
    (append-to-log (make-logged-instance obj))))