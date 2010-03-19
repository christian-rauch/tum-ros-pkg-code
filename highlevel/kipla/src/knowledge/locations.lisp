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

;;; Currently implemented keys for location designators:
;;; (on table) (on counter)
;;; (to pick-up) (to put-down)
;;; (to see) (location ?loc)
;;; (of ?obj)
;;; (for ?obj-type)

;;; TODO: Assure that only desigs are passed!
(defun sort-locations (ref-loc locations)
  (when ref-loc
    (let ((ref-jlo (etypecase ref-loc
                     (jlo:jlo ref-loc)
                     (location-designator (reference ref-loc)))))
      (sort locations (lambda (l1 l2)
                        (let ((l1-jlo (etypecase l1
                                        (jlo:jlo l1)
                                        (number (jlo:make-jlo :id l1))
                                        (string (jlo:make-jlo :name l1))))
                              (l2-jlo (etypecase l2
                                        (jlo:jlo l2)
                                        (number (jlo:make-jlo :id l2))
                                        (string (jlo:make-jlo :name l2)))))
                          (< (jlo:euclidean-distance ref-jlo l1-jlo)
                             (jlo:euclidean-distance ref-jlo l2-jlo))))))))

(defun obj-desig-location (obj-desig)
  (when (typep obj-desig 'object-designator)
    (perceived-object-pose (reference obj-desig))))

(defun loc-desig-location (loc-desig)
  (when (and (typep loc-desig 'location-designator)
             loc-desig)
    (reference loc-desig)))

(defun obj-desig-type (obj-desig)
  (when obj-desig
    (with-desig-props (type) obj-desig
      type)))

(def-fact-group location-designators
  
  (<- (loc-desig? ?desig)
    (lisp-pred typep ?desig location-designator))

  (<- (closest-location ?ref-loc ?locations ?loc)
    (bound ?ref-loc)
    (bound ?locations)
    (lisp-fun sort-locations ?ref-loc ?locations (?loc . ?_))
    (== (?loc . ?_) ?sorted-locations)
    ;; (member ?loc ?sorted-locations)
    )

  (<- (obj-desig-type ?obj ?type)
    (lisp-fun obj-desig-type ?obj ?type))

  (<- (obj-desig-location ?obj ?loc)
    (lisp-pred identity ?obj)
    (lisp-pred valid ?obj)
    (lisp-fun obj-desig-location ?obj ?loc)
    (lisp-pred identity ?loc))

  (<- (obj-desig-location ?obj ?loc)
    (desig-prop ?obj (at ?loc-desig))
    (lisp-pred identity ?loc-desig)
    (loc-desig-location ?loc-desig ?loc))

  (<- (loc-desig-location ?loc-desig ?loc)
    (lisp-fun loc-desig-location ?loc-desig ?loc)
    (lisp-pred identity ?loc))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (to reach))
    (desig-prop ?desig (location ?obj-loc))
    (closest-location ?obj-loc ("/manipulation-location-counter" "/manipulation-location-table")
                      ?loc))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (to reach))
    (desig-prop ?desig (obj ?obj))
    (obj-desig-location ?obj ?obj-loc)
    (closest-location ?obj-loc ("/manipulation-location-counter" "/manipulation-location-table")
                      ?loc))
  
  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on counter))
    (desig-prop ?desig (for ?obj))
    (obj-desig-type ?obj Jug)
    (member ?loc ("/location-counter-jug"
                  "/location-counter")))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on counter))
    (desig-prop ?desig (for ?obj))
    (obj-desig-type ?obj coke)
    (member ?loc ("/location-counter-jug"
                  "/location-counter")))
    
  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on table))
    (desig-prop ?desig (for ?obj))
    (obj-desig-type ?obj Jug)
    (member ?loc ("/location-table-jug"
                  "/location-table")))
    
  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on counter))
    (desig-prop ?desig (for ?obj))
    (obj-desig-type ?obj Mug)
    (member ?loc ("/location-counter-mug"
                  "/location-counter")))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on table))
    (desig-prop ?desig (for ?obj))
    (obj-desig-type ?obj Mug)
    (member ?loc ("/location-table-mug"
                  "/location-table")))
  
  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on counter))    
    (desig-prop ?desig (for ?obj))
    (obj-desig-type ?obj Teabox)
    (member ?loc ("/location-counter-teabox"
                  "/location-counter")))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on table))    
    (desig-prop ?desig (for ?obj))
    (obj-desig-type ?obj Teabox)
    (member ?loc ("/location-table-teabox"
                  "/location-table")))
  
  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on counter))
    (desig-prop ?desig (for ?obj))
    (obj-desig-type ?obj icetea)
    (member ?loc ("/location-counter-icetea"
                  "/location-counter")))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on table))
    (desig-prop ?desig (for ?obj))
    (obj-desig-type ?obj icetea)
    (member ?loc ("/location-table-icetea"
                  "/location-table")))
    
  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on table))
    (member ?loc ("/location-table")))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (on counter))
    (member ?loc ("/location-counter")))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (to see))
    (desig-prop ?desig (location ?perceive-loc))
    (closest-location ?perceive-loc ("/manipulation-location-counter" "/manipulation-location-table")
                       ?loc))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (to see))
    (desig-prop ?desig (obj ?obj))
    (obj-desig-location ?obj ?obj-loc)
    (closest-location ?obj-loc ("/manipulation-location-counter" "/manipulation-location-table")
                      ?loc))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (to see))
    (desig-prop ?desig (obj ?obj))
    (not (desig-prop ?obj (at ?_)))
    (member ?loc ("/manipulation-location-counter" "/manipulation-location-table")))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (jlo ?loc)))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (of ?obj))
    (lisp-fun obj-desig-location ?obj ?loc))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (jlo-list ?lo-list))
    (member ?loc ?lo-list)))
