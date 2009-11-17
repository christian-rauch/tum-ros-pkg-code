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


(in-package :desig)

(defclass designator ()
  ((timestamp :reader timestamp :initform nil
              :documentation "Timestamp of creation of reference or nil
                             if still not referencing an object.")
   (description :reader description :initarg :description
                :documentation "List of properties describing the designator.")
   (parent :reader parent :initarg :parent :initform nil
           :documentation "The parent designator, i.e. the designator
                          used to create this designator, or nil.")
   (children :accessor children :initform nil
             :documentation "List of designators this designator is the
                            parent of.")
   (valid :reader valid :initform nil
          :documentation "Returns true if the designator is valid,
                         i.e. its reference has already been computed.")
   (data :initform nil
         :documentation "Data this designator describes or nil if the
                        designator was resolved yet.")))

(defgeneric equate (desig-1 desig-2)
  (:documentation "Returns t if the two designators can be equated,
                   i.e. if they describe the same object."))

(defgeneric reference (desig)
  (:documentation "Computes and/or returns the lisp object this
                   designator references. Note: this method _MUST_ be
                   deterministic, i.e. it must always return the same
                   object."))

(defgeneric next-solution (desig)
  (:documentation "Returns a new designator that points to a different
                   object but matches the same description or nil if
                   no other solutions can be found.. This method is
                   ment for dealing with ambiguities in designator
                   descriptions."))

(defgeneric merge-designators (desig-1 desig-2)
  (:documentation "Returns a new designator with the solutions of
                   desig-1 and desig-2 merged. The soulutions of
                   desig-1 come first. Returns nil if the two
                   designators cannot be merged."))

(defmethod equate (desig-1 desig-2)
  (equal desig-1 desig-2))

(defmethod equate ((desig-1 designator) (desig-2 designator))
  ;; We can equate two designators if they have the same parent.
  (labels ((is-child (child parent) 
             "Returns t if desig-2 is a child of desig-2"
             (or (eq child parent)
                 (when parent
                   (some (curry #'is-child parent) (children child))))))
    (or (is-child desig-2 desig-1)
        (is-child desig-1 desig-2))))

(defmethod reference :after ((desig designator))
  (setf (slot-value desig 'valid) t))

(defmacro register-designator-type (type class-name)
  `(pushnew (cons ',type ',class-name) (get 'make-designator :desig-types) :key #'car))

(defun make-designator (type description &optional parent)
  (let ((desig (make-instance (cdr (assoc type (get 'make-designator :desig-types)))
                 :description description
                 :parent parent)))
    (when parent
      (push desig (children parent)))
    desig))

(defun original-desig (desig)
  "Returns the original designator, this designator was constructed
  from. I.e. the root of the designator-tree."
  (if (null (parent desig))
      desig
      (original-desig (parent desig))))

(defun youngest-children (desig)
  (when (children desig)
    (or (mapcan #'youngest-children (children desig))
        (children desig))))

;;; TODO: Make with-designators use language features. We need a
;;; transparent with-designator macro.
(defmacro with-designators (defs &body body)
  `(let* ,(mapcar (lambda (def)
                    (destructuring-bind (name (type props)) def
                      `(,name (make-designator ',type ,props))))
                  defs)
     ,@body))
