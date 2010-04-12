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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Tracing fluent
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass traced-fluent (traced-instance)
  ((name :initarg :name :reader name)
   (traced-value :initarg :traced-value :reader traced-value)))

;;; Here it is vital that we inherit from TRACING-INSTANCE-MIXIN first, so the
;;; INITIALIZE-INSTANCE :AFTER method of TRACING-INSTANCE-MIXIN is called last
;;; (after the fluent has been fully set up).
;;;
;;; NOTE #demmeln: At the moment the trace-queue would ne strictly need to be
;;; thread safe, since during execution of a plan it is accessed by the fluent
;;; only and only while the fluents lock is held. If speed is a concern,
;;; remove the lock in the queue. Maybe later we need it for simultaniously
;;; tracing and removing traced instances by writing them directly to disk
;;; from another thread (if the trace consumes too much RAM).
(defclass tracing-fluent (tracing-instance-mixin fluent)
  ((trace-queue :initarg :trace-queue
                :documentation "The fluents own thread safe queue to append
                                its traces to.")))

(defmethod print-object ((fluent tracing-fluent) stream)
  (print-unreadable-object (fluent stream :type t :identity t)
    (format stream "[Tracing Fluent]")))

(defmethod make-traced-instance ((fluent tracing-fluent))
  (make-instance 'traced-fluent
    :name (name fluent)
    :traced-value (durable-copy (value fluent))))

(defmethod append-to-trace ((traced-inst traced-fluent) fluent)
  (with-slots (trace-queue) fluent
    (cut:enqueue trace-queue traced-inst)))

;;; This was initially implemented as an :AFTER method to PULSE. But with
;;; that, pulses and the traces might happen in different orders und thus get
;;; differen timestamps.  ON-UPDATE-HOOK is called by PULSE while the fluents
;;; VALUE-LOCK is held and thus ensures a pulse and the corresponding trace
;;; cannot be intercepted by other pulses/value-changes/etc...
(defmethod on-update-hook ((fluent tracing-fluent))
  (trace-instance fluent))

(defvar *make-tracing-fluent* nil
  "Bind to not NIL if you want MAKE-FLUENT to return tracing fluents.")

(defun set-make-tracing-fluent (tracing?)
  "Pass T to let MAKE-FLUENT return TRACING-FLUENTS, NIL otherwise"
  (setf *make-tracing-fluent* tracing?))

(defun make-fluent (&rest args &key name &allow-other-keys)
  "Construct a fluent."
  (if *make-tracing-fluent*
      (apply #'make-instance 'tracing-fluent
             :trace-queue (episode-knowledge-fluent-trace-queue name)
             args)
      (apply #'make-instance 'fluent args)))
