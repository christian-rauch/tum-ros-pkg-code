
(in-package :kipla)

(defun run-demo-counter-to-table ()
  (startup-ros)
  (pick-and-place-icetea&jug-2))

(defun run-demo-table-to-counter ()
  (startup-ros)
  (pick-and-place-icetea&jug))
