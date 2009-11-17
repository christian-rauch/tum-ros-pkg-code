;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Nikolaus Demmel <demmeln@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :kipla-reasoning)

(def-fact-group time

  (<- (is-bound-time ?t)
    (is-bound ?t)
    (lisp-pred time-value-p ?t))

  ;; NOTE: (throughout a b) -> including a, including b
  ;; #demmeln: should it be excluding b?
  
  (<- (duration-includes (throughout ?t1 ?t2) (at ?a))
    (is-bound-time ?t1)
    (is-bound-time ?t2)
    (is-bound-time ?a)
    (and (<= ?t1 ?a) (<= ?a ?t2)))
  
  (<- (duration-includes (throughout ?t1 ?t2) (at ?a))
    (is-bound-time ?t1)
    (is-bound-time ?t2)
    (not (is-bound ?a))
    (== ?a ?t1))

   (<- (duration-includes (throughout ?t1 ?t2) (throughout ?a ?b))
    (is-bound-time ?t1)
    (is-bound-time ?t2)
    (is-bound-time ?a)
    (is-bound-time ?b)
    (and (<= ?t1 ?a) (<= ?a ?b) (<= ?b ?t2)))

   (<- (duration-includes (throughout ?t1 ?t2) (during ?a ?b))
    (is-bound-time ?t1)
    (is-bound-time ?t2)
    (is-bound-time ?a)
    (is-bound-time ?b)
    (and (<= ?t1 ?b) (<= ?a ?b) (<= ?a ?t2)))) 

;;; An attempt to a more general time handling follows. Turned out not
;;; so general and not at all elegant, maybe something emerges in the
;;; future...
#+nil
(
   (<- (foo ?x)
     (== ?x :foo))
  
   (<- (time-includes (throughout ?a ?b) ?t)
     (is-bound ?a) (is-bound ?b)
     (lisp-pred time-value-p ?a)
     (lisp-pred time-value-p ?b)
     (time-includes-throughout-helper ?a ?b ?t))
  
   (<- (time-includes-throughout-helper ?a ?b (throughout ?x ?y))
     (is-bound ?x) (is-bound ?y)
     (lisp-pred time-value-p ?x)
     (lisp-pred time-value-p ?y)
     (lisp-pred <= ?a ?x)
     (lisp-pred <= ?y ?b))

   (<- (time-includes-throughout-helper ?a ?b (throughout ?x ?y))
     (is-bound ?x) (not (is-bound ?y))
     (lisp-pred time-value-p ?x)
     (lisp-pred <= ?a ?x)
     (== ?b ?y))

   (<- (time-includes-throughout-helper ?a ?b (throughout ?x ?y))
     (not (is-bound ?x)) (is-bound ?y)
     (lisp-pred time-value-p ?y)
     (lisp-pred <= ?y ?b)
     (== ?a ?x))

   (<- (time-includes-throughout-helper ?a ?b (throughout ?x ?y))
     (not (is-bound ?x)) (not (is-bound ?y))
     (== ?b ?y)
     (== ?a ?x))

   (<- (time-includes-throughout-helper ?a ?b (during ?x ?y))
     (is-bound ?x) (is-bound ?y)
     (lisp-pred time-value-p ?x)
     (lisp-pred time-value-p ?y)
     (lisp-pred <= ?x ?b)
     (lisp-pred <= ?a ?y))

   (<- (time-includes-throughout-helper ?a ?b (during ?x ?y))
     (is-bound ?x) (not (is-bound ?y))
     (lisp-pred time-value-p ?x)
     (lisp-pred <= ?x ?b)
     (lisp-fun get-max-time ?y))

   (<- (time-includes-throughout-helper ?a ?b (during ?x ?y))
     (not (is-bound ?x)) (is-bound ?y)
     (lisp-pred time-value-p ?y)
     (lisp-pred <= ?a ?y)
     (lisp-fun get-min-time ?x))

   (<- (time-includes-throughout-helper ?a ?b (during ?x ?y))
     (not (is-bound ?x)) (not (is-bound ?y))
     (lisp-fun get-min-time ?x)
     (lisp-fun get-max-time ?y))

   (<- (time-includes-throughout-helper ?a ?b (at ?t))
     (is-bound ?t)
     (lisp-pred time-value-p ?t)
     (lisp-pred <= ?a ?t)
     (lisp-pred <= ?t ?b))

   (<- (time-includes-throughout-helper ?a ?b (at ?t))
     (not (is-bound ?t))
     (== ?t ?a)))