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

(in-package :cram-execution-trace)

;;; The tracing facility is kept quite general. Inherit from
;;; TRACING-INSTANCE-MIXIN if you want to trace (TRACE-INSTANCE) the state of
;;; an object by creating "snapshots" of the relevant state
;;; (MAKE-TRACED-INSTANCE) and relvant times (e.g. changes of the state) and
;;; adding those snapshots to a queue (APPEND-TO-TRACE).
;;;
;;; The snapshots should be objects of a type the
;;; inherits from TRACED-INSTANCE. Inherit form TRACED-ID-INSTANCE if you want
;;; every snapshots to include and id that is unique for EQ tracing objects.
;;;
;;; You can use a global instance trace queue where the snapshots are appended
;;; to or provide your own facilities by specializing the according methods.
;;;
;;; At the time of writing the only tracing objects are TRACING-FLUENTS and
;;; they use neither the global trace queue nor the id mechanism, so that part
;;; is unused. I keep the code in here for possible later use.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Global trace
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *instance-trace* (make-queue :name "Global Instance Trace")
  "Changes in tracing objects are put on a thread safe queue. Note that
   fluents are not traced here but in EPISODE-KNOWLEDGE objects.")

(defun get-traced-instances ()
  (list-queue-contents *instance-trace*))

(defun get-traced-timespan ()
  (let ((l (sort (get-traced-instances) #'< :key #'timestamp)))
    (values (timestamp (first l))
            (timestamp (car (last l))))))

(defun get-max-traced-time ()
  (multiple-value-bind (min max) (get-traced-timespan)
    (declare (ignore min))
    max))

(defun get-min-traced-time ()
  (multiple-value-bind (min max) (get-traced-timespan)
    (declare (ignore max))
    min))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Traced Instance
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass traced-instance ()
  ((timestamp :initform nil :initarg :timestamp :reader timestamp))
  (:documentation "Traced instances are the objects actually saved in the
                   trace. All traces have a timestamp."))

(defmethod initialize-instance :after ((instance traced-instance) &key)
  (with-slots (timestamp) instance
    (setf timestamp (or timestamp (current-timestamp)))))

(defclass traced-id-instance (traced-instance)
  ((id :initform nil :initarg :id :reader id))
  (:documentation "Traced Id Instances carry an additional id. The id in
                   traced instance that come from the same object are
                   identical. You can provied custom ids by passing :ID to
                   make-instance or use the default id which is an integer
                   where objects are tested for EQness."))

(defmethod initialize-instance :after ((instance traced-id-instance) &key)
  (with-slots (id) instance
    (setf id (or id (get-eq-id instance)))))

(defgeneric append-to-trace (traced-inst tracing-inst)
  (:documentation "Appends a traced instance to the according trace.
                   Define new methods, if you want to use specific traces
                   (queues, files, etc) for specific types.")
  (:method ((traced-inst traced-instance) tracing-inst)
    "Use global trace queue."
    (declare (ignore tracing-inst))
    (enqueue *instance-trace* traced-inst)))

(defgeneric make-traced-instance (obj)
  (:documentation "This should return an object of type TRACED-INSTNACE (or
                   subclass).  It should be a snapshot of the current state of
                   `obj'.  You will want to specialize this for subclasses of
                   tracing-instance-mixin."))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Tracing Instance
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass tracing-instance-mixin ()
  ()
  (:documentation "Use this mixin for classes you want to trace."))

;;; Always trace upon creation
(defmethod initialize-instance :after ((instance tracing-instance-mixin) &key)
  (trace-instance instance))

(defgeneric trace-instance (obj)
  (:documentation "Uses MAKE-TRACED-INSTANCE to create a trace and appends it
                   to the trace using APPEND-TO-TRACE.  You don't need to
                   specialize this if you can specialize APPEND-TO-TRACE
                   instead (or neither if you want to use the global trace
                   queue).")
  (:method ((obj tracing-instance-mixin))
    (append-to-trace (make-traced-instance obj) obj)))