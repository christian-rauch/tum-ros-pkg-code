;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Piotr Esden-Tempski <esdentem@cs.tum.edu>,
;;;                       Lorenz Moesenlechner <moesenle@cs.tum.edu>
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

(defpackage :kipla-reasoning
  (:documentation "All the prolog stuff used in kipla.")
  (:use #:common-lisp
        #:cram-utilities
        #:cram-designators
        #:cram-reasoning)
  (:export
   ;; belief-state
   #:update-belief #:clear-belief #:*believed-occasions*
   #:assert-occasion #:retract-occasion #:holds
   ;; object-belief
   #:perceived-object #:make-perceived-object
   #:perceived-object-lo-id #:perceived-object-properties
   #:perceived-object-probability #:perceived-object-desig
   #:perceived-object-timestamp #:*perceived-objects*
   #:clear-object-belief #:update-perceived-object
   #:perceived-objects-equal? #:compatible-properties
   ;; designators
   #:deisig-loc #:action-desig #:manip-desig
   #:action #:object #:location
   #:action-designator #:location-designator #:object-designator
   #:type #:mug #:icetea #:cluster #:jug #:placemat #:coke
   #:color #:black #:red
   #:at #:matches
   #:to #:reach #:on #:for #:see #:counter #:table #:of
   #:lo-id-list #:lo-id
   #:grasp #:navigate #:pose #:parked #:open #:lift #:carry #:put-down
   #:obj #:gripper #:close #:resolve-object-desig
   ;; trajectory-actions
   #:trajectory-action #:side #:trajectory-type #:stored-pose-type
   #:object-type #:hand-primitive #:end-effector-lo-id
   #:obstacles #:grasp-distance #:supporting-plane
   #:copy-trajectory-action
   ;; cop-designators
   #:cop-desig-query-info #:cop-desig-query-info-object-classes
   #:cop-desig-query-info-object-ids #:cop-desig-query-info-poses
   #:cop-desig-query-info-matches
   #:cop-desig-location-info #:cop-desig-location-info-poses
   #:cop-desig-info #:cop-desig-info-designator #:cop-desig-info-query
   #:cop-desig-info-location
   ;; perceived-object
   
   )
  (:import-from #:cram-language
                #:task-tree-node #:task-tree-node-path
                #:task-tree-node-code #:code-parameters
                #:code-task #:status #:goal #:logged-fluent #:value
                #:timestamp #:logged-value #:get-logged-timespan
                #:time-value-p #:make-fluent #:name #:current-timestamp
                #:*task-tree* #:flatten-task-tree #:get-logged-instances)
  (:import-from #:alexandria
                #:curry #:rcurry #:compose))

(defpackage :kipla
  (:documentation "Kitchen Planner and coordinator")
  (:use #:common-lisp
        #:roslisp
        #:cram-utilities
        #:cram-language
        #:cram-designators
        #:cram-process-modules
        #:kipla-reasoning)
  (:export #:log-msg)
  (:import-from #:alexandria
                #:with-gensyms #:curry #:rcurry)
  (:import-from #:cram-reasoning
                #:prolog)
  (:shadowing-import-from
   #:cpl
   #:< #:> #:+ #:- #:* #:/ #:= #:eq #:eql #:not))
