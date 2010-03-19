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

(in-package :kipla)

(def-top-level-plan nav-to (obj)
  (pursue
    (run-process-modules)
    (with-designators ((loc (location `((on ,obj))))
                       (see-loc (location `((to see) (location ,loc)))))
      (sleep 0.5)
      (achieve `(loc Robot ,see-loc)))))

(def-top-level-plan perceive-cluster ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (cluster (object `((type cluster) (at ,counter)))))
      (sleep 0.5)
      (perceive cluster))))

(def-top-level-plan perceive-mug ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location '((on table))))
                       (mug (object `((type mug) (at ,table)))))
      (sleep 0.5)
      (perceive mug))))

(def-top-level-plan perceive-mug-no-nav ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on table))))
                       (mug (object `((type mug) (at ,counter)))))
      (sleep 0.5)
      (pm-execute 'perception mug))))

(def-top-level-plan perceive-icetea-no-nav ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (mug (object `((type icetea) (at ,counter)))))
      (sleep 0.5)
      (pm-execute 'perception mug))))

(def-top-level-plan perceive-objects ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location '((on table))))
                       (obj (object `((type object) (at ,table)))))
      (sleep 0.5)
      (perceive-all obj))))

(def-top-level-plan re-perceive-obj (desig)
  (pursue
    (run-process-modules)
    (seq
      (sleep 0.5)
      (pm-execute 'perception desig))))

(def-top-level-plan perceive-jug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on table))))
                       (jug (object `((type jug) (at ,counter)))))
      (sleep 0.5)
      (perceive jug))))

(def-top-level-plan perceive-icetea ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location '((on table))))
                       (icetea (object `((type icetea) (at ,table)))))
      (sleep 0.5)
      (perceive icetea))))

(def-top-level-plan perceive-placemat ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location '((on table))))
                       (placemat (object `((type placemat) (at ,table)))))
      (sleep 0.5)
      (perceive placemat))))

(def-top-level-plan grasp-mug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (mug (object `((type mug) (at ,counter)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,mug :right)))))

(def-top-level-plan grasp-jug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (jug (object `((type jug) (at ,counter)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,jug :right)))))

(def-top-level-plan grasp-icetea ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location '((on table))))
                       (icetea (object `((type icetea) (at ,table)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,icetea :right)))))

(def-top-level-plan grasp-cluster ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location '((on table))))
                       (cluster (object `((type cluster) (at ,table)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,cluster :right))
      (achieve `(arms-at ,(make-designator 'action '((type trajectory) (to show) (side :right)))))
      (sleep 5)
      (achieve `(object-placed-at ,cluster ,(make-designator 'location `((of ,cluster))))))))

(def-top-level-plan putdown-obj (obj)
  (pursue
    (run-process-modules)
    (with-designators ((loc (location `((on table) (for ,obj)))))
      (sleep 0.5)      
      (achieve `(object-placed-at ,obj ,loc)))))

(def-top-level-plan perceive-icetea&mug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location `((on counter))))
                       (icetea (object `((type icetea) (at ,counter))))
                       (table (location `((on table))))
                       (mug (object `((type mug) (at ,table)))))
      (perceive icetea)
      (perceive mug))))

(def-top-level-plan pick-and-place-jug ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type jug) (at ,table))))
                       (counter (location `((on table) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-top-level-plan pick-and-place-obj (obj)
  (pursue
    (run-process-modules)
    (with-designators ((counter (location `((on table) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-top-level-plan pick-and-place-icetea&jug ()
  (say "I will bring the icetea and the jug to the counter.")
  (pursue
    (run-process-modules)
    (with-designators ((table (location `((on table))))
                       (jug (object `((type jug) (at ,table))))
                       (icetea (object `((type icetea) (at ,table))))
                       (counter-jug (location `((on counter) (for ,jug))))
                       (counter-icetea (location `((on counter) (for ,icetea)))))
      (sleep 0.5)
      (achieve `(object-in-hand ,jug :left))
      (achieve `(arms-at ,(make-designator 'action '((type trajectory) (pose open) (side :left)))))
      (achieve `(object-in-hand ,icetea :right))
      (achieve `(object-placed-at ,icetea ,counter-icetea))
      (achieve `(object-placed-at ,jug ,counter-jug)))))

(def-top-level-plan pick-and-place-icetea&jug-2 ()
  (say "I will bring the icetea and the jug to the table.")
  (pursue
    (run-process-modules)
    (with-designators ((counter (location `((on counter))))
                       (jug (object `((type jug) (at ,counter))))
                       (icetea (object `((type icetea) (at ,counter))))
                       (table-jug (location `((on table) (for ,jug))))
                       (table-icetea (location `((on table) (for ,icetea)))))
      (sleep 0.5)
      (achieve `(object-in-hand ,icetea :right))
      (achieve `(arms-at ,(make-designator 'action '((type trajectory) (pose open) (side :right)))))
      (achieve `(object-in-hand ,jug :left))
      (achieve `(object-placed-at ,icetea ,table-icetea))
      (achieve `(object-placed-at ,jug ,table-jug))
      (achieve `(arm-parked :both)))))

(def-top-level-plan pick-and-place-icetea ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type icetea) (at ,table))))
                       (counter (location `((on counter) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-top-level-plan pick-and-place-coke()
  (pursue
    (run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type coke) (at ,table))))
                       (counter (location `((on counter) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-top-level-plan pick-and-place-mug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location `((on table))))
                       (obj (object `((type mug) (at ,counter))))
                       (table (location `((on table) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,table)))))

(def-top-level-plan pick-and-place-on-placemat ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type jug) (at ,table))))
                       (placemat (object `((type placemat) (at ,table))))
                       (dest-loc (location `((of ,placemat)))))
      (setf placemat (perceive placemat))
      (achieve `(loc ,obj ,dest-loc)))))

(def-top-level-plan put-down (obj)
  (pursue
    (run-process-modules)
    (with-designators ((loc (location `((on counter) (for ,obj)))))
      (format t "putting down to loc ~a~%" (reference loc))
      (sleep 0.5)
      (achieve `(object-placed-at ,obj ,loc)))))

(def-top-level-plan park-arms ()
  (pursue
    (run-process-modules)
    (seq
      (sleep 0.5)
      (achieve '(arm-parked :both)))))

(def-top-level-plan test-reach ()
  (pursue
    (run-process-modules)
    (loop for i from 1 to 100 do
         (with-designators ((loc (location `((on table))))
                            (obj (object `((type jug) (at ,loc)))))
           (achieve `(object-in-hand ,obj :left))
           (clear-belief)
           (sleep 10)))))

(def-top-level-plan right-carry ()
  (pursue
    (run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((carry-desig (action '((type trajectory) (to carry) (side :right)))))
        (achieve `(arms-at ,carry-desig))))))

(def-top-level-plan both-open ()
  (pursue
    (run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((open-desig (action '((type trajectory) (pose open) (side :both)))))
        (achieve `(arms-at ,open-desig))))))