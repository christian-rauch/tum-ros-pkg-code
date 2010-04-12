(in-package :cpl-tests)

(def-suite tasks :in language)

(in-suite tasks)


;;;; WITH-TASK-HIERARCHY

(eval-when (:compile-toplevel :execute)
  (defmacro assert-set-difference-empty
      (A B &body (format-control . format-args))
    `(let ((diff (set-difference ,A ,B)))
       (assert (null diff) ()
               ,(concatenate 'string
                             "~@<" format-control " ~2I~:_~{~S~^, ~}.~@:>")
               ,@format-args
               diff))))

(defmacro with-task-hierarchy (hierarchy definitions &body body)
  (flet ((parse-hierarchy (clauses)
           ;; Returns a hash-table mapping from task-name to its
           ;; children.
           (let ((map (make-hash-table))
                 (seen-parents '())
                 (seen-childs '()))
             (mapc #'(lambda (clause)
                       (destructuring-bind (parent -> . childs) clause
                         (assert (eq -> '->)
                                 () "Bogus hierarchy clause: ~S" clause)
                         (assert (not (nth-value 1 (gethash parent map)))
                                 () "Duplicate hierarchy clause for ~S" parent)
                         (setf (gethash parent map) childs)
                         (push parent seen-parents)
                         (dolist (child childs)
                           (push child seen-childs))))
                   clauses)
             (assert-set-difference-empty seen-childs seen-parents
               "The following task names appear on the right hand ~
                side in the hierarchy specification yet you forgot ~
                to provide corresponding left hand sides for them:")
             (values (nreverse seen-parents) map)))
         (parse-definitions (defs)
           ;; Returns a hash-table mapping from task-name to its body
           ;; forms.
           (let ((map (make-hash-table)))
             (mapc #'(lambda (def)
                       (destructure-case def
                         ((:task task-name . task-body)
                          (setf (gethash task-name map) task-body))))
                   defs)
             (values (hash-table-keys map) map)))
         (generate-task-constructor (&key name childs form)
           (let ((thread-fun
                  `#'(lambda ()
                       ;;; FIXME: Maybe add :PARENT / :CHILDS
                       ;;; parameter to (MAKE-INSTANCE 'TASK ...)
                       ,@(loop for child in childs
                               collect `(cpl-impl::register-child
                                         *current-task* ,child)
                               collect `(setf (slot-value ,child 'cpl-impl::parent-task)
                                              ,name))
                       ,form)))
             `(make-instance 'task
                             :name ',name
                             :thread-fun ,thread-fun                           
                             :run-thread nil))))
    (multiple-value-bind (task-names hierarchy-map)
        (parse-hierarchy hierarchy)
      (multiple-value-bind (task-names* definition-map)
          (parse-definitions definitions)
        (assert-set-difference-empty task-names task-names*
          "Missing body definitions for tasks:")
        (assert-set-difference-empty task-names* task-names
          "Missing hierarchy specifications for tasks:")
        (let ((tasks-ready (gensym "TASKS-READY+"))
              (n-tasks (length task-names)))
          `(with-synchronization-barrier (,tasks-ready ,(1+ n-tasks))
             (let ,task-names
               ;; Make the tasks, and assign them to lexical variables
               ;; named like them. Do not actually run them yet.
               ,@(loop for task-name in task-names
                       for task-body   = (gethash task-name definition-map)
                       for task-childs = (gethash task-name hierarchy-map)
                       collect
                       `(setq ,task-name
                              ,(generate-task-constructor
                                :name task-name
                                :childs task-childs
                                :form `(block ,task-name
                                         ;; Wait for all other tasks
                                         ;; to be set up properly. 
                                         (enter-barrier ,tasks-ready)
                                         ,@task-body))))
               ;; Execute the tasks now. We have to defer execution to
               ;; this point, because only now all the lexical
               ;; variables contain their TASK instances.
               ,@(loop for task-name in task-names
                       collect `(cpl-impl::execute ,task-name
                                                   :ignore-no-parent t))
               (enter-barrier ,tasks-ready)
               ,@body)))))))


;;;; Some more task-related utilities

(defun wait-until (predicate &rest tasks)
  (dolist (task tasks)
    (wait-for (funcall predicate task))))

(defun become (&rest states)
  (setq states (mappend #'ensure-list states))
  (dolist (state states)
    (check-type state cpl-impl::status-indicator))
  #'(lambda (task)
      (fl-funcall #'member (status task) states)))

(defparameter +dead+
  '(:failed :succeeded :evaporated))

(defparameter +alive+
  '(:created :running :waiting :suspended))

(defun task-status (task)
  (value (status task)))

;;; This is a macro so to profit from 5AM:IS's DWIMish failure
;;; reporting feature.
(defmacro has-status (task status)
  `(is (eq ,status (task-status ,task))))

(defun suspend (task)
  (cpl-impl::suspend task))

(defun wake-up (task)
  (cpl-impl::wake-up task))

(defun terminate (task termination-status)
  (cpl-impl::terminate task termination-status))

(defun evaporate (&rest tasks)
  (dolist (task tasks)
    (terminate task :evaporated)))

(defun evaporate-and-wait (&rest tasks)
  (apply #'evaporate tasks)
  (apply #'wait-until (become :evaporated) tasks))


;;;; The actual tests

;;; Child succeeds ==> parent unaffected

(define-cram-test tasks--basics.1 ()
    "Child awakes before parent."
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 0.05))
       (:task B (sleep 0.025)))
    (wait-until (become +dead+) A B)
    (has-status A :succeeded)
    (has-status B :succeeded)))

;;; Parent succeeds => child evaporates

(define-cram-test tasks--basics.2 ()
    "Parent awakes before child."
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 0.025))
       (:task B (sleep 0.05)))
    (wait-until (become +dead+) A B)
    (has-status A :succeeded)
    (has-status B :evaporated)))

;;; Parent suspends => child suspends

(define-cram-test tasks--suspend.1 ()
    "Suspend parent, make sure child becomes suspended, too.
     Then wake up parent, make sure child awakes, too."
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (suspend A) (wait-until (become :suspended) A B)
    (wake-up A) (wait-until (become :running) A B)
    (evaporate-and-wait A B)))

;;; Parent & child suspended, child wake-up => error

(define-cram-test tasks--suspend.2 ()
    "Like TASKS-SUSPEND.1, but wake up the child first.
     This violates the invariant that a child must not be running
     without its parent running, too. Hence we actually expect an
     error on the attempt to wake up."
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (suspend A) (wait-until (become :suspended) A B)
    (signals error
      (wake-up B))
    (evaporate-and-wait A B)))

;;; Child suspends => parent unaffected

(define-cram-test tasks--suspend.3 ()
    "Suspend child, make sure parent is not affected.
     Then wake up the child again."
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (suspend B)  (wait-until (become :suspended) B)
    (sleep 0.01) (has-status A :running)
    (wake-up B)  (wait-until (become :running) B)
    (evaporate-and-wait A B)
    (pass)))

;;; Parent evaporates => child evaporates

(define-cram-test tasks--terminate.1 ()
    "Evaporate parent, make sure child gets evaporated, too."
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (evaporate A)
    (wait-until (become :evaporated) A B)
    (pass)))

;;; Child evaporates, parent running => error

(define-cram-test tasks--terminate.2 ()
    ""
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (signals error
      (evaporate B))
    (evaporate-and-wait (become :evaporated) A B)))

;;; Parent fails => child evaporates

(define-cram-test tasks--failure.1 ()
    "Parent fails, make sure child gets evaporated."
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (terminate A :failed)
    (wait-until (become :failed) A)
    (wait-until (become :evaporated) B)
    (pass)))

;;; Child fails => parent fails

(define-cram-test tasks--failure.2 ()
    "Child fails, make sure parent fails, too."
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (terminate B :failed)
    (wait-until (become :failed) A B)
    (pass)))
