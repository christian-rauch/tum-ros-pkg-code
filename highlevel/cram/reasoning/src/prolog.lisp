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


(in-package :crs)

(defun unify (lhs rhs &optional bdgs)
  "Unifiy two forms."
  (let ((lhs (substitute-vars lhs bdgs))
        (rhs (substitute-vars rhs bdgs)))
    (cond ((or (and (atom lhs) (atom rhs)
                    (eql lhs rhs))
               (and (not lhs) (not rhs)))
           (values bdgs t))
          ((is-var lhs)
           (add-bdg lhs rhs bdgs))
          ((is-var rhs)
           (add-bdg rhs lhs bdgs))
          ((is-segform lhs)
           (match-segvar lhs rhs bdgs #'unify))
          ((is-segform rhs)
           (match-segvar rhs lhs bdgs #'unify))
          ((and (consp lhs) (consp rhs))
           (multiple-value-bind (new-bdgs matched?)
               (unify (car lhs) (car rhs) bdgs)
             (when matched?
               (unify (cdr lhs) (cdr rhs) new-bdgs)))))))

(defun prove-one (goal binds)
  (let ((handler (get-prolog-handler (car goal))))
    (or (when handler (apply handler binds (cdr goal)))
        (lazy-mapcan (lambda (clause-match)
                       (prove-all (fact-clauses (car clause-match))
                                  (cdr clause-match)))
                     (get-matching-clauses goal binds)))))

(defun prove-all (goals binds)
  (cond ((null goals)
         (list binds))
        (t
         (lazy-mapcan (lambda (goal-1-binds)
                        (prove-all (cdr goals) goal-1-binds))
                      (prove-one (car goals) binds)))))

(defun get-matching-clauses (query binds)
  (nreverse (reduce-fact-definitions
             (lambda (result fact-def)
               (let ((fact-def (rename-vars fact-def)))
                 (multiple-value-bind (new-bdgs matched?)
                     (unify (fact-head fact-def) query binds)
                   (if matched?
                       (cons (cons fact-def new-bdgs) result)
                       result)))))))

(defun rename-vars (fact-definition)
  (let* ((replacements nil)
         (new-fact-header
          (map-tree (lambda (q)
                      (if (and (is-var q) (not (is-unnamed-var q)))
                          (or (cdr (assoc q replacements))
                              (cdar (push (cons q (gen-var))
                                          replacements)))
                          q))
                    (car fact-definition))))
    (cons new-fact-header (sublis replacements (cdr fact-definition)))))

(defun filter-bindings (form bdgs &optional initial-bdgs)
  "Removes all bindings from bdgs that are not used in form."
  (cond ((is-var form)
         (let ((val (var-value form bdgs)))
           (if (eq val form)
               initial-bdgs
               (add-bdg form val initial-bdgs))))
        ((or (not form) (atom form))
         initial-bdgs)
        (t
         (filter-bindings (cdr form) bdgs
                          (filter-bindings (car form) bdgs initial-bdgs)))))

(defun prolog (query &optional (binds nil))
  (lazy-mapcar (rcurry (curry #'filter-bindings query) binds)
               (prove-one query binds)))

