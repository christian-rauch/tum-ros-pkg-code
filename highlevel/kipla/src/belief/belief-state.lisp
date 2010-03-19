;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :kipla-reasoning)

(defgeneric update-belief (data &optional reference-object)
  (:documentation "Updates the belief state and returns the anchored
                   instance of data."))

(defvar *believed-occasions* (make-fluent :name 'believed-occasions))

(defun occasion-equal-test (lhs rhs)
  (if (and (typep lhs 'designator)
           (typep rhs 'designator))
      (desig-equal lhs rhs)
      (eql lhs rhs)))

(defun clear-belief ()
  (setf (value *believed-occasions*) nil)
  (clear-alpha-network))

(defun assert-occasion (occ)
  (kipla:log-msg :info "Asserting occasion `~a'" occ)
  (pushnew occ (value *believed-occasions*)
           :test (rcurry #'pat-match-p :test #'occasion-equal-test))
  (rete-assert occ))

(defun retract-occasion (occ)
  (kipla:log-msg :info "Retracting occasion `~a'" occ)
  (setf (value *believed-occasions*)
        (remove-if (lambda (current)
                     (when (pat-match-p occ current :test #'occasion-equal-test)
                       (rete-retract current)
                       t))
                   (value *believed-occasions*))))

(defun holds (occ)
  (force-ll (rete-holds occ #'occasion-equal-test)))
