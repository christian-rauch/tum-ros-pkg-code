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

(defvar *alpha-network* (make-instance 'alpha-node :parent nil :key nil))

(defun make-alpha-node (&rest params &key (class 'alpha-node) key parent &allow-other-keys)
  (labels ((remove-key (key seq)
             (when seq
               (if (eq (car seq) key)
                   (remove-key key (cddr seq))
                   (cons (car seq) (remove-key key (cdr seq)))))))
    (let ((node (apply #'make-instance class (remove-key :class params))))
      (when parent
        (setf (gethash key (slot-value parent 'children)) node)))))

(defun get-alpha-node (pattern &optional
                       (parent-node *alpha-network*)
                       (unmatched pattern))
  "Creates all alpha-nodes that represent the pattern. 'node'
   specifies the parent node that already matched parts of the
   pattern. 'unmatched' contains the unmatched parts of the pattern."
  (assert pattern)
  (destructuring-bind (key . unmatched) unmatched
    (let ((node (alpha-node-matching-child parent-node pattern key (not unmatched))))
      (cond (unmatched
             (get-alpha-node pattern node unmatched))
            (t
             ;; Update the pattern of the node since it must be a
             ;; valid path specifyer.
             (unless (alpha-memory-node-pattern node)
               (setf (slot-value node 'pattern)
                     pattern))
             node)))))

(defun alpha-node-matching-child (node pattern key memory-node?)
  (or (gethash key (slot-value node 'children))
      (if memory-node?
          (make-alpha-node
           :class 'alpha-memory-node :key key :parent node
           :pattern (when (consp pattern)
                      pattern))
          (make-alpha-node :key key :parent node))))

(defun alpha-node-wildcard-child (node)
  (gethash '? (slot-value node 'children)))

(defun rete-assert (fact)
  (input *alpha-network* fact :assert))

(defun rete-retract (fact)
  (input *alpha-network* fact :retract))

(defun alpha-network-size (&optional (node *alpha-network*))
  (1+ (loop for v being the hash-values in (slot-value node 'children)
         summing (alpha-network-size v))))
