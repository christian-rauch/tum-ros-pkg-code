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


(in-package :cpl-impl)

(defmacro def-fluent-operator (&whole w name args &body body)
  "def-fluent-operator allows to define fluent operators. It creates a
   new function with the given name. When it is called with a fluent
   in its parameter list, a fluent network is returned, with the
   function body as its calculator. Otherwise, the body is called
   directly.

   We need one special case. Some operators need to always pulse the
   returned fluent net. This is solved by havin a second value. When
   the body returns a non-nil second value, the fluent net is always
   pulsed, otherwise only when the fluent net value has changed."
  (multiple-value-bind (body decls docstring) 
      (parse-body body :documentation t :whole w)
    (with-gensyms (fl-args)
      `(defun ,name (&rest ,fl-args)
         ,docstring
         (labels ((,name ,args ,@decls ,@body))
           (let ((fluents (remove-if-not (of-type 'fluent) ,fl-args)))
             (if fluents 
                 (call-as-fluent-operator #',name ,fl-args 
                                          :fluents fluents 
                                          :name ',name)
                 (apply #',name ,fl-args))))))))

(defun call-as-fluent-operator (function args 
                                &key (fluents (required-argument))
                                     (name    (required-argument)))
  "The meat of DEF-FLUENT-OPERATOR."
  (let* ((fl-name (format-gensym "FN-~A" name)) 
         (result-fluent (make-fluent
                         :name fl-name
                         ;; calculate value after registering at 
                         ;; dependency-fluents
                         :value nil
                         :dependencies fluents))
         (weak-result-fluent (tg:make-weak-pointer result-fluent)))
    (without-termination
      (dolist (fluent fluents)
        (register-update-callback
         fluent fl-name
         (lambda ()
           (without-termination
             (let ((result-fluent (tg:weak-pointer-value weak-result-fluent)))
               (cond (result-fluent
                      ;; We need to find out if the fluent has been pulsed by setf.
                      ;; Otherwise we need to pulse it by hand. This is a hack again,
                      ;; but currently I see no better solution.
                      (multiple-value-bind (new-value pulse?) (apply function args)
                        (let ((old-pulse-count
                               (with-lock-held ((slot-value result-fluent 'value-lock))
                                 (slot-value result-fluent 'pulse-count))))
                          (setf (value result-fluent) new-value)
                          (when (and pulse?
                                     (= old-pulse-count
                                        (with-lock-held ((slot-value result-fluent 'value-lock))
                                          (slot-value result-fluent 'pulse-count))))
                            (pulse result-fluent)))))
                     (t
                      ;; Do garbage-collection when reference to
                      ;; fluent is no longer valid.
                      (dolist (fluent fluents)
                        (remove-update-callback fluent fl-name)))))))))
      (setf (value result-fluent) (apply function args)))
    result-fluent))

(macrolet ((wrap (fn args)
             ;; This solution is not the fastest one and not the
             ;; because of the use of mapcar, which causes at least
             ;; two iterations over args.
             `(apply #',fn (mapcar #'value ,args))))

  (def-fluent-operator fl< (&rest args)
    (wrap < args))

  (def-fluent-operator fl> (&rest args)
    (wrap > args))

  (def-fluent-operator fl= (&rest args)
    (wrap = args))

  (def-fluent-operator fl-eq (&rest args)
    (wrap eq args))

  (def-fluent-operator fl-eql (&rest args)
    (wrap eql args))
  
  (def-fluent-operator fl+ (&rest args)
    (wrap + args))

  (def-fluent-operator fl- (&rest args)
    (wrap - args))

  (def-fluent-operator fl* (&rest args)
    (wrap * args))

  (def-fluent-operator fl/ (&rest args)
    (wrap / args)))

(def-fluent-operator fl-not (arg)
  (not (value arg)))

;;; AND and OR cannot be implemented as macros for fluent. All
;;; previous operators return whether a fluent or the value, depending
;;; on whether at least one fluent is in the arguments or not. That
;;; means, variables must be expanded before deciding which return
;;; value to use, which requires expansion of values. Thus, AND and OR
;;; have to behave slightly different for fluents and we name them
;;; differently.

(def-fluent-operator fl-and (&rest args)
  "The and-operator for fluents. It is fundamentally different to the
   definition of common-lisp's and in that it is not implemented as a
   macro. That means, all args are evaluated when using fl-and."
  (cond ((null args)
         t)
        ((cdr args)
         (and (value (car args))
              (apply #'fl-and (cdr args))))
        (t
         (value (car args)))))

(def-fluent-operator fl-or (&rest args)
  "The or-operator for fluents. For more information on why it is a
   function please refere to the documentation of fl-and."
  (when args
    (or (value (car args))
        (apply #'fl-or (cdr args)))))

(def-fluent-operator fl-pulsed (&rest args)
  "Returns true and is invoked whenever one of its
   argument fluents gets pulsed."
  (when args
    (values t t)))

(def-fluent-operator fl-funcall (fun &rest args)
  "Generic fluent-operator. Applys args to function whenever a
   fluent in args changes."
  (when args
    (apply fun (mapcar #'value args))))