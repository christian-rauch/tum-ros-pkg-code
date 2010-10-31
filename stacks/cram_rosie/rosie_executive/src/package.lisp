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

(defpackage :rosie-executive
  (:nicknames :kipla :rex)
  (:use #:cpl
        #:cram-designators
        #:cram-plan-library
        #:designators-ros
        #:cram-roslisp-common
        #:table-costmap)
  (:import-from #:cram-plan-knowledge
                #:clear-belief)
  (:shadowing-import-from #:table-costmap
                          #:name))

(defpackage :rosie-executive-reasoning
    (:nicknames :kipla-reasoning :rex-reasoning)
  (:use #:common-lisp
        #:crs
        #:roslisp
        #:desig
        #:cram-utilities
        #:location-costmap
        #:table-costmap
        #:cram-plan-knowledge
        #:cram-plan-library
        #:cram-reasoning
        #:cram-designators
        #:cram-roslisp-common
        #:designators-ros
        #:perception-pm)
  (:import-from #:cpl-impl #:value)
  (:import-from #:cram-plan-library
                #:object-picked-up
                #:object-in-hand-failure))
