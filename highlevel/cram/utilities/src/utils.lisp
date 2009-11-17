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


(in-package :cut)

(defun map-tree (fun tree)
  (labels ((%mapcar* (fun seq &optional result-cons)
             (cond ((null seq)
                    nil)
                   ((atom seq)
                    (setf (cdr result-cons) (funcall fun seq)))
                   (t
                    (let ((new-cons (cons (funcall fun (car seq)) nil)))
                      (cond (result-cons
                             (setf (cdr result-cons) new-cons)
                             (%mapcar* fun (cdr seq) new-cons))
                            (t
                             (%mapcar* fun (cdr seq) new-cons)
                             new-cons)))))))
    (cond ((listp tree)
           (%mapcar* (lambda (elem)
                       (if (listp elem)
                           (map-tree fun elem)
                           (funcall fun elem)))
                     tree))
          (t (funcall fun tree)))))

(defmacro pop-if! (pred lst)
  "Destructively modifies the sequence. The first item for which pred
  returns a non-nil value is removed."
  (with-gensyms (pred-sym)
    `(labels ((worker (pred lst)
                (when (cdr lst)
                  (if (funcall pred (cadr lst))
                      (setf (cdr lst) (cddr lst))
                      (worker pred (cdr lst))))))
       (let ((,pred-sym ,pred))
         (when ,lst
           (if (funcall ,pred-sym (car ,lst))
               (if (cdr ,lst)
                   (setf (car ,lst) (cadr ,lst)
                         (cdr ,lst) (cddr ,lst))
                   (setf ,lst nil))
               (worker ,pred-sym ,lst))))
       ,lst)))
