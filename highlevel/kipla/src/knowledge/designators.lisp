;;;
;;; Copyright (C) 2009 by Nikolaus Demmel <demmeln@cs.tum.edu>
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

(macrolet ((def-desig-accessor (slot &optional (predicate-name nil))
             (let* ((predicate-name (or predicate-name slot))
                    (name (format-symbol t "DESIG-~a" predicate-name))
                    (var (format-symbol t "?~a" predicate-name)))
               `(<- (,name ?desig ,var)
                  (bound ?desig)
                  (get-slot-value ?desig ,slot ,var)))))
  (def-fact-group designator-accessors
    ;; DESIG-TIMESTAMP
    (def-desig-accessor timestamp)
    ;; DESIG-DESCRIPTION
    (def-desig-accessor description)
    ;; DESIG-VALID
    (def-desig-accessor valid)
    ;; DESIG-VALUE
    (def-desig-accessor data value)))

(def-fact-group designators
  (<- (desig-prop ?desig (?prop-name ?prop))
    (bound ?desig)
    (bound ?prop-name)
    (lisp-fun desig-prop-value ?desig ?prop-name ?prop)
    (lisp-pred identity ?prop))
  
  (<- (desig-equal ?d1 ?d2)
    (bound ?d1)
    (bound ?d2)
    (lisp-pred desig-equal ?d1 ?d2)))
