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

(defun is-var (var)
  "Predicate that returns a non-nil value if the symbol is a
   variable."
  (and (symbolp var)
       (eq (elt (symbol-name var) 0)
           #\?)))

(defun is-unnamed-var (var)
  "Returns true if the variable is the unnamed variable (?_)"
  (eq var '?_))

(defun is-segvar (var)
  "Predicate that returns a non-nil value if the symbol is a
   segment-variable."
  (and (symbolp var)
       (eq (elt (symbol-name var) 0) #\!)
       (eq (elt (symbol-name var) 1) #\?)))

(defun is-segform (form)
  "Predicates that returns a non-nil value when form is a segform,
   i.e. its car is a segvar."
  (and (consp form)
       (is-segvar (car form))))

(defun var-name (var)
  "Always returns the variable name or throws an error (i.e. extracts
   the variable name from a segform."
  (cond ((is-var var)
         var)
        ((is-segvar var)
         (intern (subseq (symbol-name var) 1)))
        (t
         (error "No variable."))))

(defun substitute-vars (form bdgs)
  (map-tree (rcurry #'var-value bdgs) form))

(defun var-value (var binds)
  "Returns the value of a variable. Note: The variable is simplified,
  i.e. if its value is a variable the value of that variable is
  returned."
  (if (is-var var)
      (let ((value (assoc var binds)))
        (if (not value)
            var
            (substitute-vars (cdr value) binds)))
      var))

(defun gen-var (&optional base)
  "Returns a newly generated unique var."
  (gensym (or base "?VAR-")))

(defun is-genvar (var)
  "Returns true if the symbol has been generated with gen-var."
  (and (symbolp var)
       (not (symbol-package var))))

(defun add-bdg (var val bdgs)
  "Adds a variable binding to the bindings list."
  (if (is-unnamed-var var)
      (values bdgs t)
      (let ((old-bdg (assoc var bdgs)))
        (if old-bdg
            (and (equal (cdr old-bdg) val)
                 (values bdgs t))
            (values (cons `(,var . ,val) bdgs) t)))))

(defun match-segvar (pat seq bdgs continuation)
  "Matches a segvar. The continuation is a recursively called function
   that matches the forms after the segform. (TODO: explain better)"
  (labels ((collect-segval (&optional (seq seq) (content nil))
             (multiple-value-bind (new-bdgs matched?)
                 (funcall continuation (cdr pat) seq bdgs)
               (if (and (not matched?) seq)
                   (collect-segval (cdr seq) `(,@content ,(car seq)))
                   (values new-bdgs content)))))
    (if (endp (cdr pat))
        (add-bdg (var-name (car pat)) seq bdgs)
        (multiple-value-bind (new-bdgs content)
            (collect-segval)
          (when new-bdgs
            (add-bdg (var-name (car pat)) content new-bdgs))))))

(defun pat-match (pat seq &optional (bdgs nil) &rest rest)
  "Match a pattern."
  (unless (listp bdgs)
    (push bdgs rest)
    (setf bdgs nil))
  (destructuring-bind (&key (test #'eql)) rest
    (cond ((is-var pat)
           (add-bdg (var-name pat) seq bdgs))
          ((and (atom pat) (atom seq)
                (funcall test pat seq))
           (values bdgs t))
          ((is-segform pat)
           (match-segvar pat seq bdgs (rcurry #'pat-match :test test)))
          ((and (consp pat) (consp seq))
           (multiple-value-bind (new-bdgs matched?)
               (pat-match (car pat) (car seq) bdgs :test test)
             (and matched? (pat-match (cdr pat) (cdr seq) new-bdgs
                                      :test test)))))))

(defun pat-match-p (pat seq &optional (bdgs nil) &rest rest)
  "Pattern matching predicate. Does not return bdgs but only T or nil,
  indicating if the pattern matches."
  (cdr (multiple-value-list (apply #'pat-match pat seq bdgs rest))))

(defun vars-in (pat)
  "Returns the variables in pattern."
  (labels ((worker (&optional (pat pat) (vars nil))
             (if (null pat)
                 vars
                 (worker (cdr pat)
                         (cond ((listp (car pat))
                                (worker (car pat) vars))
                               ((is-var (car pat))
                                (cons (car pat) vars))
                               (t
                                vars))))))
    (worker)))

(defmacro with-vars-bound (pat seq &body body)
  (with-gensyms (bdgs)
    `(let ((,bdgs (pat-match ',pat ,seq)))
       (unless ,bdgs
         (error "Pattern does not match."))
       (let ,(mapcar (lambda (var)
                       `(,var (var-value ',var ,bdgs)))
                     (vars-in pat))
         ,@body))))
