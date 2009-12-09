
(in-package :kipla)

(def-plan nav-to (obj)
  (pursue
    (run-process-modules)
    (with-designators ((loc (location `((on ,obj))))
                       (see-loc (location `((to see) (location ,loc)))))
      (sleep 0.5)
      (achieve `(loc Robot ,see-loc)))))

(def-plan perceive-cluster ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (cluster (object `((type cluster) (at ,counter)))))
      (sleep 0.5)
      (perceive cluster))))

(def-plan perceive-mug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (mug (object `((type mug) (at ,counter)))))
      (sleep 0.5)
      (perceive mug))))

(def-plan perceive-jug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on table))))
                       (jug (object `((type jug) (at ,counter)))))
      (sleep 0.5)
      (perceive jug))))

(def-plan perceive-icetea ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (icetea (object `((type icetea) (at ,counter)))))
      (sleep 0.5)
      (perceive icetea))))

(def-plan perceive-placemat ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location '((on table))))
                       (placemat (object `((type placemat) (at ,table)))))
      (sleep 0.5)
      (perceive placemat))))

(def-plan grasp-mug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (mug (object `((type mug) (at ,counter)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,mug :right)))))

(def-plan grasp-jug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (jug (object `((type jug) (at ,counter)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,jug :right)))))

(def-plan grasp-icetea ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (jug (object `((type icetea) (at ,counter)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,jug :right)))))

(def-plan putdown-obj (obj)
  (pursue
    (run-process-modules)
    (with-designators ((loc (location `((on table) (for ,obj)))))
      (sleep 0.5)      
      (achieve `(object-placed-at ,obj ,loc)))))

(def-plan pick-and-place-jug ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type jug) (at ,table))))
                       (counter (location `((on table) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-plan pick-and-place-obj (obj)
  (pursue
    (run-process-modules)
    (with-designators ((counter (location `((on table) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-plan pick-and-place-icetea&jug ()
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

(def-plan pick-and-place-icetea&jug-2 ()
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

(def-plan pick-and-place-icetea ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type icetea) (at ,table))))
                       (counter (location `((on counter) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-plan pick-and-place-coke()
  (pursue
    (run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type coke) (at ,table))))
                       (counter (location `((on counter) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-plan pick-and-place-mug ()
  (pursue
    (run-process-modules)
    (with-designators ((counter (location `((on table))))
                       (obj (object `((type mug) (at ,counter))))
                       (table (location `((on table) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,table)))))

(def-plan pick-and-place-on-placemat ()
  (pursue
    (run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type jug) (at ,table))))
                       (placemat (object `((type placemat) (at ,table))))
                       (dest-loc (location `((of ,placemat)))))
      (setf placemat (perceive placemat))
      (achieve `(loc ,obj ,dest-loc)))))

(def-plan put-down (obj)
  (pursue
    (run-process-modules)
    (with-designators ((loc (location `((on counter) (for ,obj)))))
      (format t "putting down to loc ~a~%" (reference loc))
      (sleep 0.5)
      (achieve `(object-placed-at ,obj ,loc)))))

(def-plan park-arms ()
  (pursue
    (run-process-modules)
    (seq
      (sleep 0.5)
      (achieve '(arm-parked :both)))))

(def-plan test-reach ()
  (pursue
    (run-process-modules)
    (loop for i from 1 to 100 do
         (with-designators ((loc (location `((on table))))
                            (obj (object `((type jug) (at ,loc)))))
           (achieve `(object-in-hand ,obj :left))
           (clear-belief)
           (sleep 10)))))

(def-plan right-carry ()
  (pursue
    (run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((carry-desig (action '((type trajectory) (to carry) (side :right)))))
        (achieve `(arms-at ,carry-desig))))))

(def-plan both-open ()
  (pursue
    (run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((open-desig (action '((type trajectory) (pose open) (side :both)))))
        (achieve `(arms-at ,open-desig))))))