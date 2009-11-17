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


(in-package :cpl)

(defmacro def-fluent-operator (name args &body body)
  "def-fluent-operator allows to define fluent operators. It creates a
   new function with the given name. When it is called with a fluent
   in its parameter list, a fluent network is returned, with the
   function body as its calculator. Otherwise, the body is called
   directly.

   We need one special case. Some operators need to always pulse the
   returned fluent net. This is solved by havin a second value. When
   the body returns a non-nil second value, the fluent net is always
   pulsed, otherwise only when the fluent net value has changed."
  (let* ((documentation (when (stringp (car body))
                          (car body)))
         (body (if documentation
                   (cdr body)
                   body)))
    (with-gensyms (fl-args)
      `(defun ,name (&rest ,fl-args)
         ,documentation
         (labels ((,name ,args
                    ,@body))
           (macrolet ((for-each-fluent ((arg seq) &body body)
                        `(loop for ,arg in ,seq
                            when (typep ,arg 'fluent) do (progn ,@body)))
                      (has-fluent? (seq)
                        `(find-if (lambda (arg)
                                    (typep arg 'fluent))
                                  ,seq)))

             (cond ((has-fluent? ,fl-args)
                    (let* ((fluent-net-name (gensym (format nil "FN-~a" ',name)))
                           (result-fluent (make-fluent
                                           :name fluent-net-name
                                           :value nil ; calculate value after registering at dependency-fluents
                                           :dependencies (loop for fl in ,fl-args when (typep fl 'fluent) collecting fl)))
                           (weak-result-fluent (tg:make-weak-pointer result-fluent)))
                      (terminate-protect
                        (for-each-fluent (fluent ,fl-args)
                                         (register-update-callback
                                          fluent fluent-net-name
                                          (lambda ()
                                            (terminate-protect
                                              (let ((result-fluent (tg:weak-pointer-value weak-result-fluent)))
                                                (cond (result-fluent
                                                       ;; We need to find out if the fluent has been pulsed by setf.
                                                       ;; Otherwise we need to pulse it by hand. This is a hack again,
                                                       ;; but currently I see no better solution.
                                                       (with-lock-held ((slot-value result-fluent 'value-lock))
                                                         (multiple-value-bind (new-value pulse?) (apply #',name ,fl-args)
                                                           (let ((old-pulse-count (slot-value result-fluent 'pulse-count)))
                                                             (setf (value result-fluent)
                                                                   new-value)
                                                             (when (and pulse?
                                                                        (= old-pulse-count
                                                                           (slot-value result-fluent 'pulse-count)))
                                                               (pulse result-fluent))))))
                                                      (t
                                                       ;; Do garbage-collection when reference to fluent is no longer valid.
                                                       (for-each-fluent (fl ,fl-args)
                                                                        (remove-update-callback fl fluent-net-name)))))))))
                        (setf (value result-fluent) (apply #',name ,fl-args)))
                      result-fluent))
                   (t
                    (apply #',name ,fl-args)))))))))

(macrolet ((def-cl-wrapper (name args)
             ;; This solution is not the fastest one and not the
             ;; because of the use of mapcar, which causes at least
             ;; two iterations over args.
             `(apply #',(intern (symbol-name name) :common-lisp)
                     (mapcar #'value ,args))))

  (def-fluent-operator < (&rest args)
    (def-cl-wrapper < args))

  (def-fluent-operator > (&rest args)
    (def-cl-wrapper > args))

  (def-fluent-operator = (&rest args)
    (def-cl-wrapper = args))

  (def-fluent-operator eq (&rest args)
    (def-cl-wrapper eq args))

  (def-fluent-operator eql (&rest args)
    (def-cl-wrapper eql args))

    (def-fluent-operator + (&rest args)
    (def-cl-wrapper + args))

  (def-fluent-operator - (&rest args)
    (def-cl-wrapper - args))

  (def-fluent-operator * (&rest args)
    (def-cl-wrapper * args))

  (def-fluent-operator / (&rest args)
    (def-cl-wrapper / args)))

(def-fluent-operator not (arg)
  (cl:not (value arg)))

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
  (cond ((not args)
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

(def-fluent-operator pulsed (&rest args)
  "Returns true and is invoked whenever one of its
   argument fluents gets pulsed."
  (when args
    (values t t)))

(def-fluent-operator fl-funcall (fun &rest args)
  "Generic fluent-operator. Applys args to function whenever a
   fluent in args changes."
  (when args
    (apply fun (mapcar #'value
                       args))))
