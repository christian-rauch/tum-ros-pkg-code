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


(in-package :crs)

(defclass production-node ()
  ((beta-node :reader production-beta-node :initarg :beta-node)
   (tokens :reader production-tokens :initarg :tokens :initform nil)
   (callback :reader production-callback :initarg :callback
             :documentation "The callback of the production. It is
                             called with each token passed by the beta
                             node and the corresponding operation.")))

(defun make-production-node (beta-node callback)
  (let ((new-node (make-instance 'production-node
                    :beta-node beta-node
                    :callback callback)))
    (pushnew new-node (slot-value beta-node 'connections))
    (mapc (rcurry callback :assert) (slot-value beta-node 'matches))
    new-node))

(defmethod propagate ((node production-node) token operation)
  (with-slots (tokens callback) node
    (case operation
      (:assert
       (push token tokens))
      (:retract
       (remove token tokens)))
    (funcall callback token operation)))

(defmethod gc-node ((node production-node))
  (setf (slot-value (production-beta-node node) 'connections)
        (remove node (slot-value (production-beta-node node) 'connections)))
  (gc-node (production-beta-node node)))

(defvar *productions* (make-hash-table :test 'eq))

(defstruct production-definition
  (name nil)
  (var-bindings nil)
  (body nil)
  (node nil)
  (callbacks nil))

(defmacro def-production (name var-bindings &body definition)
  "Defines a new production. 'name' is the name of the production,
   'var-bindings' the list of variables that should be passed to the
   callback when the production gets triggered."
  `(progn
     (when (gethash ',name *productions*)
       (remhash ',name *productions*)
       (warn ,(format nil "Redefining production '~a'." name)))
     (setf (gethash ',name *productions*)
           (make-production-definition :name ',name
                                       :var-bindings ',var-bindings
                                       :body ',definition))))

(defun production-node-type (definition)
  "Returns the type of definition. It can (currently) be either :prolog or :beta-join.
   It is used to select the correct node type. (In the future, there
   should also be a :beta-not type.:"
  (cond ((find-prolog-fact definition)
         :prolog)
        (t
         :beta-join)))

(defun create-production-network (production var-bindings callback)
  (labels ((get-definition-variables (definition)
             "Returns the variables of a single definition, with a
             position designator."
             (loop
                for field in definition
                for i from 0
                when (is-var field) collecting (cons field i)))
           (get-variable-accessors (production-definition &optional (index 0) (accessors nil))
             "Collects accessor functions to get the value of a
              variable out of a token. The accessor function gets two
              parameters, an index and the current token."
             (cond ((null production-definition)
                    accessors)
                   (t
                    (let ((variables (get-definition-variables (car production-definition))))
                      (get-variable-accessors
                       (cdr production-definition)
                       (1+ index)
                       (nconc 
                        (loop for (var . var-index) in variables
                           unless (assoc var accessors) collecting (cons var (lambda (caller-index token)
                                                                               (elt (alpha-memory-node-pattern
                                                                                     (elt token (- caller-index index)))
                                                                                    var-index))))
                        accessors))))))
           (get-fact (definition)
             (loop for field in definition
                if (is-var field) collecting '?
                else collecting field))
           (add-beta-node (definition parent-beta-node callback)
             (case (production-node-type definition)
               (:prolog
                (make-prolog-node parent-beta-node callback))
               (:beta-join
                (let ((alpha-node (get-alpha-node (get-fact definition))))
                  (make-beta-join-node parent-beta-node alpha-node callback)))))
           (build-network (definitions accessors &optional (parent-beta-node nil) (index 0))
             "Builds up the network and returns the last beta node"
             (let ((beta-node (add-beta-node (car definitions) parent-beta-node
                                             (lambda (token)
                                               (loop
                                                  for field in (car definitions)
                                                  for value in (alpha-memory-node-pattern (token-wme token))
                                                  when (and (is-var field)
                                                            (not (eql value (funcall (cdr (assoc field accessors))
                                                                                     index token))))
                                                  return nil
                                                  finally (return t))))))
               (if (cdr definitions)
                   (build-network (cdr definitions) accessors beta-node (1+ index))
                   beta-node))))
    (let* ((accessors (get-variable-accessors production))
           (last-beta-node (build-network production accessors)))
      (make-production-node
       last-beta-node
       (lambda (token operation)
         (apply callback operation
                (loop for var in var-bindings
                   collecting (let ((var-accessor (cdr (assoc var accessors))))
                                (unless var-accessor
                                  (error (format nil "Variable ~a not defined in production-body."
                                                 var)))
                                (funcall var-accessor (1- (length token)) token)))))))))

(defun register-production-handler (production-name callback)
  (let ((production (gethash production-name *productions*)))
    (unless production
      (error "Production unknown."))
    (pushnew callback (production-definition-callbacks production))
    (unless (production-definition-node production)
      (setf (production-definition-node production)
            (create-production-network (production-definition-body production)
                                       (production-definition-var-bindings production)
                                       (lambda (&rest args)
                                         (loop for callback in (production-definition-callbacks production)
                                            do (apply callback args))))))
    (production-definition-node production)))
