;;;
;;; Copyright (c) 2009, Lars Kunze <kunzel@cs.tum.edu>
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


(in-package :liswip)

(define-foreign-library libpl
    (:unix (:or "libpl.so.5.6.58" "libpl.so"))
    (t (:default "libpl")))

(defparameter *pl-user-module* "user")

(use-foreign-library libpl)

;;;; types
(defctype atom_t :unsigned-long)
(defctype term_t :unsigned-long)
(defctype qid_t :unsigned-long)
(defctype functor_t :unsigned-long)
(defctype predicate_t :pointer)
(defctype module_t :pointer)
(defctype fid_t :unsigned-long)

;;;; term-type constants
;; PL_unify_term() arguments
(defconstant PL-VARIABLE 1) ;; nothing 
(defconstant PL-ATOM 2)     ;; const char
(defconstant PL-INTEGER 3)  ;; int
(defconstant PL-FLOAT 4)    ;; double 
(defconstant PL-STRING 5)   ;; const char 
(defconstant PL-TERM 6)
(defconstant PL-LIST 7)     ;; list

;;;; embedding
(defun pl-init (argv)
  (pl-initialise (length argv) (foreign-alloc :string :initial-contents argv)))

(defcfun "PL_initialise" :int
  "Initialises the SWI-Prolog"
    (argc  :int)
    (argv  :pointer))

(defcfun "PL_cleanup" :int
  "This function performs the reverse of PL_initialise()"
   (status :int))
  
(defcfun "PL_halt" :int
  "Cleanup the Prolog environment using PL_cleanup() and calls exit() with the status argument"
  (status :int))

(defcfun "PL_new_module" module_t
  (name atom_t))

;;;; call-back
(defconstant PL-Q-NORMAL 2)
(defconstant PL-Q-NODEBUG 4)
(defconstant PL-Q-CATCH_EXCEPTION 8)

;; foreign context frames
(defcfun "PL_open_foreign_frame" fid_t)

(defcfun "PL_rewind_foreign_frame" :void
  (cid fid_t))

(defcfun "PL_close_foreign_frame" :void
  (cid fid_t))

(defcfun "PL_discard_foreign_frame" :void
  (cid fid_t))

;; finding predicates
(defcfun "PL_pred" predicate_t
  "Return a handle to a predicate for the specified name/arity in the given module"
  (functor functor_t)
  (module module_t))

(defcfun "PL_predicate" predicate_t
  "Return a handle to a predicate for the specified name/arity in the given module"
  (name :string)
  (arity :int)
  (module :string))

;; call-back
(defcfun "PL_open_query" qid_t
  (module module_t)
  (flags :int)
  (predicate predicate_t)
  (term term_t))

(defcfun "PL_next_solution" :int
  (qid qid_t))

(defcfun "PL_close_query" :void
  (qid qid_t))

(defcfun "PL_cut_query" :void
  (qid qid_t))

;; simplified
(defcfun "PL_call" :int
  (term term_t)
  (module module_t))

(defcfun "PL_call_predicate" :int
  (module module_t)
  (debug :int)
  (predicate predicate_t)
  (term term_t))

;;;; term-references
;; creating and destroying term-refs
(defcfun "PL_new_term_refs" term_t
  "Return n new term references. The first term-reference is returned. The others are t+1, t+2, etc."
  (n :int))

(defcfun "PL_new_term_ref" term_t
  "Return a fresh reference to a term")

;; constants
(defcfun "PL_new_atom" atom_t
  (chars :string))

(defcfun "PL_atom_chars" :string
  (atom atom_t))

(defcfun "PL_new_functor" functor_t
  (name atom_t)
  (arity :int))

(defcfun "PL_functor_arity" :int
  (f functor_t))

(defcfun "PL_functor_name" atom_t
  (f functor_t))

;; assign to term-references
(defcfun "PL_put_atom_chars" :void
  "Put an atom in the term-reference constructed from the 0-terminated string"
  (term  term_t)
  (chars :string))

(defcfun "PL_put_string_chars" :void
  (term term_t)
  (chars :string))

(defcfun "PL_put_float" :void
  (term  term_t)
  (val :double))

(defcfun "PL_put_integer" :void
  (term  term_t)
  (val :long))

(defcfun "PL_put_term" :void
  (t1 term_t)
  (t2 term_t))

(defcfun "PL_cons_functor" :void
  (t1 term_t)
  (f functor_t)
  (a1 term_t)
  (a2 term_t)
  (a3 term_t))

(defcfun "PL_cons_functor_v" :void
 (ht1 term_t)
 (f functor_t)
 (t2 term_t))

;; verify types
(defcfun "PL_term_type" :int
  (term term_t))

(defcfun "PL_is_list" :int
  (term term_t))

(defcfun "PL_get_list" :int
  (lst term_t)
  (head term_t)
  (rest term_t))

;; get c-values from prolog terms
(defcfun "PL_get_integer" :int
  (term term_t)
  (val :pointer))

(defcfun "PL_get_float" :int
  (term term_t)
  (val :pointer))

(defcfun "PL_get_atom_chars" :int
  (term term_t)
  (chars :pointer))

(defcfun "PL_get_arg" :int
  "If term1 is compound and index is between 1 and arity (including), assign term2 with a term-reference to the argument"
  (index :int)
  (term1 term_t)
  (term2 term_t))

(defcfun "PL_get_name_arity" :int
  (term term_t)
  (atom :pointer)
  (arity :pointer))

(defcfun "PL_get_chars" :int
  (term term_t)
  (s :pointer)
  (flags :unsigned-int))

(defcfun "PL_get_functor" :int
  (term term_t)
  (f :pointer))

(defcfun "PL_copy_term_ref" term_t
  (term term_t))


(defvar *xsd-float* "http://www.w3.org/2001/XMLSchema#float")
(defvar *xsd-int* "http://www.w3.org/2001/XMLSchema#int")

(defun get-term-value (term)
  (with-foreign-object (val :pointer)
    (let ((term-type (pl-term-type term)))
      (cond ((or (eql term-type pl-integer) (is-owl-integer term))
             (if (eql term-type pl-integer)
                 (progn
                   (pl-get-integer term val)
                   (mem-ref val :int))
                 (get-owl-type term)))
            ((or (eql term-type pl-float) (is-owl-float term))
             (if (eql term-type pl-float)
                 (progn
                   (pl-get-float term val)
                   (mem-ref val :double))
                 (get-owl-type term)))
            ((eql term-type pl-term) ;; compound term
             (if (not (eql (pl-is-list term) 0)) 
                 (get-terms-from-list term) ;; if term is a prolog list, transform to lisp list
                 (progn                     ;; other compound terms are returned as string
                   (pl-get-chars term val 127)
                   (mem-ref val :string))))
            ((eql term-type pl-string) 
             (pl-get-chars term val 127)
             (mem-ref val :string))
            ((eql term-type pl-atom)
             (pl-get-chars term val 127)
             (intern (mem-ref val :string)))
            (t
             (pl-get-chars term val 127)
             (mem-ref val :string))))))

(defun get-terms-from-list (termlist)
  (let ((head (pl-new-term-ref))
        (lst  (pl-copy-term-ref termlist))
        (res  (list)))
    (loop until (eql (pl-get-list lst head lst) 0)
       do
         (setf res (append res (list (get-term-value head)))))
    res))

(defun get-owl-type (term)
   (let ((type (pl-new-term-ref))
         (val (pl-new-term-ref)))
     (pl-get-arg 1 term type)
     (pl-get-arg 2 type val)
     (read-from-string (string (get-term-value val)))))

(defun is-owl-integer (term)
  (is-owl-type *xsd-int* term))

(defun is-owl-float (term)
  (is-owl-type *xsd-float* term))

(defun is-owl-type (type term)
  (with-foreign-object (val :pointer)
    (if (eql (pl-term-type term) pl-term)
        (progn
          (pl-get-chars term val 127)
          (search (concatenate (quote string) "literal(type(" type) (mem-ref val :string)))
        nil)))

(defun put-term-value (term val &optional vars)
  (cond ((var? val)
         (progn
           (when (not (gethash val vars))
             (setf (gethash val vars) (pl-new-term-ref)))
           (pl-put-term term (gethash val vars))))
        ((listp val)
         (let* ((functor-terms (pl-query val vars))
                (functor (car functor-terms))
                (terms (car (cdr functor-terms))))
           (pl-cons-functor-v term functor terms)))
        (t
         (when val
           (cond ((floatp val)
                  (pl-put-float term (coerce val 'double-float)))
                 ((integerp val)
                  (pl-put-integer term val))
                 ((stringp val)
                  ;;(pl-put-atom-chars term val))
                  (pl-put-string-chars term val))
                  ;;
                  ;; needed for string handling,
                  ;; but causes errors when rdf_has predicate is
                  ;; called with string instead of an atom 
                 (t
                  (pl-put-atom-chars term (string val))))))))

(defun var? (x)
  (and (symbolp x)
       (eql (char (symbol-name x) 0) #\?)))

(defun query-var (var expr)
  (multiple-value-bind
        (success binds)
      (pl-q expr)
    (if success
        (values success (value-of var (car binds)))
        nil)))

(defun query-vars (vars expr)
  (multiple-value-bind
        (success binds)
      (pl-q expr)
    (values-list
     (append
      (list success)
      (mapcan #'(lambda(x)
                  (list (value-of x (car binds))))vars)))))

(defun value-of (var binds)
  (let ((bdg (assoc var binds)))
    (cond ((null bdg) (error "var not bound"))
          (T
           (cdr bdg)))))

;; takes lisp expr, builds prolog expr, calls prolog, and retrieves answer
(defun pl-q (expr)
  (let* ((fid (pl-open-foreign-frame))
         (vars (make-hash-table))
         (functor-terms (pl-query expr vars))
         (functor (car functor-terms))
         (terms (car (cdr functor-terms)))
         (pred (pl-pred functor (pl-new-module (pl-new-atom *pl-user-module*)))))
    (let ((qid (pl-open-query (null-pointer) PL-Q-NORMAL pred terms))
          (r (list))
          (success nil))
      (loop until (eql (pl-next-solution qid) 0)
         do
           (let ((sol (list)))
             (setf success t)
             (maphash #'(lambda (k v)
                          (setf sol (append sol
                                            (list (cons k
                                                        (get-term-value v)))))) vars)
             (setf r (append r (list sol)))))
      (pl-close-query qid)
      (pl-close-foreign-frame fid)
      (values success r))))
  
;; parses lisp expression expr and constructs query by using the C interface
(defun pl-query (expr vars) 
  (case (car expr)
    (and (pl-query-and (cdr expr) vars))
    (or (pl-query-or (cdr expr) vars))
    (not (pl-query-not (cdr expr) vars))
    (t (pl-query-simple (car expr) (cdr expr) vars))))

;; handle and expressions
(defun pl-query-and (clauses vars)
  (let* ((num (length clauses)))
    (cond ((>= num 2)
           (let* ((and-functor (pl-new-functor (pl-new-atom ",") 2))
                  (and-terms (pl-new-term-refs 2))
                  (functor-terms1 (pl-query (nth 0 clauses) vars))
                  (functor1 (car functor-terms1))
                  (terms1 (car (cdr functor-terms1)))
                  (functor-terms2 nil))
             (if (eql num 2)
                 (setf functor-terms2 (pl-query (nth 1 clauses) vars));; 2 terms
                 (setf functor-terms2 (pl-query-and (cdr clauses) vars)));; more than 2 terms
             (let* ((functor2 (car functor-terms2))
                    (terms2 (car (cdr functor-terms2))))
               (pl-cons-functor-v and-terms  functor1 terms1)
               (pl-cons-functor-v (+ and-terms 1) functor2 terms2)
               (list and-functor and-terms))))
          (t
           nil))))

;; handle or expressions
(defun pl-query-or (clauses vars)
  (let* ((num (length clauses)))
    (cond ((>= num 2)
           (let* ((or-functor (pl-new-functor (pl-new-atom ";") 2))
                  (or-terms (pl-new-term-refs 2))
                  (functor-terms1 (pl-query (nth 0 clauses) vars))
                  (functor1 (car functor-terms1))
                  (terms1 (car (cdr functor-terms1)))
                  (functor-terms2 nil))
             (if (eql num 2)
                 (setf functor-terms2 (pl-query (nth 1 clauses) vars));; 2 terms
                 (setf functor-terms2 (pl-query-or (cdr clauses) vars)));; more than 2 terms
             (let* ((functor2 (car functor-terms2))
                    (terms2 (car (cdr functor-terms2))))
               (pl-cons-functor-v or-terms  functor1 terms1)
               (pl-cons-functor-v (+ or-terms 1) functor2 terms2)
               (list or-functor or-terms))))
          (t
           nil))))

;; handle not expressions
(defun pl-query-not (clauses vars)
  (let* ((num (length clauses)))
    (cond ((eql num 1)
           (let* ((not-functor (pl-new-functor (pl-new-atom "not") 1))
                  (not-term (pl-new-term-ref))
                  (functor-terms1 (pl-query (car clauses) vars))
                  (functor1 (car functor-terms1))
                  (terms1 (car (cdr functor-terms1))))
             (pl-cons-functor-v not-term functor1 terms1)
             (list not-functor not-term)))
          (t
           nil))))
            
;; handle simple expressions, i.e. prolog predicates  
(defun pl-query-simple (pred args vars)
  (let* ((arity (length args))
         (terms (pl-new-term-refs arity))
         (functor (pl-new-functor (pl-new-atom (string pred)) arity)))
    (loop for i from 0 to (- arity 1)
          do (put-term-value (+ terms i) (nth i args) vars))
    (list functor terms)))
