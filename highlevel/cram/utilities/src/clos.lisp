
(in-package :cut)

(define-method-combination hooks (&key (hook-combination #'list))
  ((methods *))
  "Every method should be qualified by a symbol. All matching methods
   are executed and the results are combined using `hook-combination'
   which returns the final result of the method. The
   `hook-combination' function gets every method result as a
   parameter."
  `(funcall ,hook-combination
          ,@(loop for m in methods
               collecting `(call-method ,m))))
