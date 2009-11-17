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
;;; Thread Safe Queue
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass ts-queue ()
  ((head :initform nil :type list)
   (tail :initform nil :type cons)
   (lock)
   (name :initform "" :initarg name :type string
         :documentation "Name is mainly for debugging locks (whostate in with-lock-held)"))
  (:documentation "Simple thread safe queue implemented as cons list."))

(defmethod initialize-instance :after ((queue ts-queue) &key &allow-other-keys)
  (with-slots (lock name) queue
    (setf lock (make-recursive-lock :name (format nil "Lock for Thread Safe Queue ~a" name)))))

(defun make-ts-queue (&optional (name nil name-passed))
  (apply #'make-instance 'ts-queue (and name-passed (list :name name))))

(defmacro with-ts-queue-lock-held ((queue reason) &body body)
  `(with-slots (lock name) ,queue
     (with-lock-held (lock :whostate
                           (format nil "Thread Safe Queue ~a: ~a"
                                   name ,reason))
       ,@body)))

(defun ts-queue-clear (queue)
  (with-ts-queue-lock-held (queue "clear")
    (with-slots (head tail)
        queue
      (setf head nil
            tail nil))))

(defun ts-queue->list (queue)
  "Returns a freshly consed list with the elements in the queue."
  (with-ts-queue-lock-held (queue "queue->list")
    (with-slots (head) queue
      (copy-list head))))

(defun ts-queue-empty? (queue)
  (with-ts-queue-lock-held (queue "empty?")
    (with-slots (head) queue
      (not head))))

(defun ts-enqueue (queue x)
  "Appends x at the end of the queue and returns x"
  (with-ts-queue-lock-held (queue "enqueue")
    (with-slots (head tail) queue
      (setf tail (last (nconc tail (list x)))
            head (or head tail))))
  x)

(defun ts-dequeue (queue)
  "Removes the first element in the queue and returns it.
   Returns t as a second value if element is returned.
   Returns nil as a second value if queue was empty."
  (with-ts-queue-lock-held (queue "dequeue")
    (with-slots (head tail) queue
      (if head
          (let ((val (car head)))
            (setf head (cdr head)
                  tail (and head tail))
            (values val t))
          (values nil nil)))))