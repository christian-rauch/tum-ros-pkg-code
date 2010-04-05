;;;
;;; Copyright (c) 2009, Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package :crs-tests)

(def-suite prolog :in reasoning)

(defmacro test-prolog (name &body tests)
  (let ((count 0)
        (name (format-symbol t "PROLOG-~a" name)))
    (flet ((next-name ()
             (format-symbol t "~a-~a" name (incf count))))
      `(progn
         ,@(mapcar (lambda (test)
                     (destructuring-bind (test query result) test
                       (case test
                         (:prolog
                          `(test ,(next-name)
                             (is (equal ',result
                                        (force-ll (prolog ',query))))))
                         (:prolog-probe
                          `(test ,(next-name)
                             (let ((answers (force-ll (prolog ',query))))
                               (is-true (not (null answers)))
                               (dolist (bdg answers)
                                 (dolist (x ',result)
                                   (is (equal (cdr x) (var-value (car x) bdg))))))))
                         (t (error "Invalid test form.")))))
                   tests)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Prolog handlers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-suite prolog-handlers :in prolog)

(in-suite prolog-handlers)

(test-prolog and-handler
  (:prolog (and (== ?x a) (== ?x a) (== ?x ?y))
           (((?y . a) (?x . a))))
  (:prolog (and (and) (and))
           (nil))
  (:prolog (and (fail))
           nil)
  (:prolog (and (and) (and) (fail))
           nil)
  (:prolog (and)
           (nil)))

(test-prolog or-handler
  (:prolog (or)
           nil)
  (:prolog (or (== a a) (== b a) (== a b))
           (nil))
  (:prolog (or (fail) (and))
           (nil))
  (:prolog (or (fail) (fail))
           nil))

(test-prolog not-handler
  (:prolog (not (fail))
           (nil))
  (:prolog (not (and))
           nil)
  (:prolog (not (p-equals-np)) ;; not is negation by failiure only
           (nil)))

(test-prolog lisp-fun
  (:prolog (and (== (a b c) ?x)
                (lisp-fun append (a b) (c) ?x))
           (((?x . (a b c)))))
  (:prolog (lisp-fun + 1 2 ?x)
           (((?x . 3)))))

(test-prolog lisp-pred
  (:prolog (lisp-pred symbolp a)
           (nil))
  (:prolog (lisp-pred atom (a b))
           nil)
  (:prolog (and (== ?x (foo bar))
                (lisp-pred equal (foo bar) ?x))
           (((?x . (foo bar))))))

(test-prolog bound-handler
  (:prolog (bound ?x)
           nil)
  (:prolog (bound a)
           (nil))
  (:prolog (and (== ?x a)
                (bound ?x))
           (((?x . a))))
  (:prolog (and (== ?x ?y)
                (bound ?x))
           nil)
  (:prolog (and (== ?x (?y))
                (bound ?x))
           (((?x . (?y))))))

(test-prolog ground-handler
  (:prolog (ground ?x)
           nil)
  (:prolog (ground a)
           (nil))
  (:prolog (and (== ?x a)
                (ground ?x))
           (((?x . a))))
  (:prolog (and (== ?x ?y)
                (bound ?x))
           nil)
  (:prolog (and (== ?x (?y))
                (ground ?x))
           nil))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Predefined prolog facts
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-suite prolog-facts :in prolog)

(in-suite prolog-facts)

(test-prolog unification
  (:prolog (== ?x a)
           (((?x . a)))))

(test-prolog bin-preds
  (:prolog (< 1 2) (nil))
  (:prolog (< 3 2) nil)
  (:prolog (> 1 0) (nil))
  (:prolog (> 0 1) nil)
  (:prolog (>= 0 0) (nil))
  (:prolog (>= 0 1) nil)
  (:prolog (<= 0 0) (nil))
  (:prolog (<= 1 0) nil))

(test-prolog member
  (:prolog (member ?x (a b c))
           (((?x . a)) ((?x . b)) ((?x . c))))
  (:prolog (member ?x ())
           nil))

(defclass dummy ()
  ((a :initform :a)))

(test-prolog clos-utils
  (:prolog (instance-of ?t ?o)
           ())
  (:prolog-probe (and (instance-of dummy ?obj)
                      (get-slot-value ?obj a ?a)) 
                 ((?a . :a)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (instance-of dummy ?obj)
                      (== ?a :a)) ;; test with ?obj bound
                 ((?a . :a)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (instance-of ?type ?obj)) 
                 ((?type . dummy)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (get-slot-value ?obj a :a)
                      (== ?a :a))
                 ((?a . :a)))
  (:prolog (and (instance-of dummy ?obj)
                (get-slot-value ?obj a :b))
           ())
  (:prolog-probe (and (instance-of dummy ?obj)
                      (slot-value ?obj a ?a))
                 ((?a . :a)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (slot-value ?obj a :a)
                      (== ?a :a))
                 ((?a . :a)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (slot-value ?obj a :b)
                      (slot-value ?obj a ?b))
                 ((?b . :b))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; General tests
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-suite prolog)

(def-fact-group prolog-tests
  (<- (fact1 ?f)
    (== ?x (1 2))
    (== ?x (?f . ?_)))
  (<- (fact2 ?x ?y)
    (== ?y (1 2))
    (member ?x ?y)))

(test-prolog rename-variables-bug
  (:prolog (fact1 ?x) (((?x . 1)))) ;; used to fail while next one passed
  (:prolog (fact1 ?y) (((?y . 1)))))

(test-prolog 1
  (:prolog (fact2 ?a ?b)
           (((?B 1 2) (?A . 1)) ((?B 1 2) (?A . 2)))))


