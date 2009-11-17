;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Lorenz Moesenlechner <moesenle@cs.tum.edu>
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
                     (number ref-loc)
                     (location-designator (reference ref-loc)))))
      (sort locations (lambda (l1 l2)
                        (< (jlo:euclidean-distance ref-jlo l1)
                           (jlo:euclidean-distance ref-jlo l2)))))))

(defun obj-desig-location (obj-desig)
  (when (typep obj-desig 'object-designator)
    (perceived-object-lo-id (reference obj-desig))))

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
    (is-bound ?ref-loc)
    (is-bound ?locations)
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
    (desig-prop ?desig (lo-id ?loc)))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (of ?obj))
    (lisp-fun obj-desig-location ?obj ?loc))

  (<- (desig-loc ?desig ?loc)
    (loc-desig? ?desig)
    (desig-prop ?desig (lo-id-list ?lo-list))
    (member ?loc ?lo-list)))
