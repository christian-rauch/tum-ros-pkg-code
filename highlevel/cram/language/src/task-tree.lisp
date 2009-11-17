;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;;                     Nikolaus Demmel <demmeln@cs.tum.edu>
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


(in-package :cpl)

(declaim (optimize (debug 3)))

;;; Represents code sexp is the sexp of the code, function is a
;;; compiled function and tasks is the list of tasks, in reverse
;;; order.
(defstruct code
  "Represents a piece of code that can be replaced. It contains the
  sexp as well as a function object that can be executed. Further, it
  contains the top-level task and all tasks in the lexical environment
  of the top-level task. This can be used to get the variable bindings
  of a specific task without the need to walk the task tree."
  sexp
  function
  task
  parameters)

(defstruct task-tree-node
  (code nil)
  (code-replacements (list))
  (parent nil)
  (children nil)
  (path nil))

;;; Define our own pretty print method, to avoid infinit recursion when
;;; *print-circle* is nil. It is similar to the general struct pretty printer
;;; but doesn't print the parent slot.
(defmethod print-object ((object task-tree-node) stream) 
  (format stream "~<#(~;~W ~:@_:CODE ~@_~W~:@_:CODE-REPLACEMENTS ~@_~W~:@_:PATH ~@_~W~:@_:CHILDREN ~@_~W~;)~:>"
	  (list 'task-tree-node
	        (and (task-tree-node-code object) :some)
            (and (task-tree-node-code-replacements object) :some)
            (task-tree-node-path object)
            (length (task-tree-node-children object)))))

(defvar *current-path* nil "Contains the current path (in reverse order).")
(defvar *task-tree* nil)
(defvar *current-task-tree-node* nil)

(defmacro with-task-tree-node ((&key (path-part (error "Path parameter is required."))
                                     sexp lambda-list parameters)
                               &body body)
  "Executes a body under a specific path. Sexp, lambda-list and parameters are optional."
  (with-gensyms (task)
    `(let* ((*current-path* (cons ,path-part *current-path*))
            (*current-task-tree-node* (ensure-tree-node *current-path*)))
       (declare (special *current-path* *current-task-tree-node*))
       (let ((,task (make-task :sexp ',(or sexp body)
                               :function (lambda ,lambda-list
                                           ,@body)
                               :parameters ,parameters)))
         (execute ,task)
         (join-task ,task)))))

(defmacro replaceable-function (name lambda-list parameters path-part
                                &body body)
  "Besides the repalcement of simple code parts defined with 'with-task-tree-node',
   it is necessary to also pass parameters to the replaceable code
   parts. For that, replaceable functions can be defined. They are not
   real functions, i.e. they do change any symbol-function or change
   the lexical environment. 'name' is used to mark such functions in the
   code-sexp. More specifically, the sexp is built like follows:
   `(replaceable-function ,name ,lambda-list ,@body).
   The 'parameters' parameter contains the values to call the function with."
  `(with-task-tree-node (:path-part ,path-part
                         :sexp `(replaceable-function ,',name ,',lambda-list . ,',body)
                         :lambda-list ,lambda-list
                         :parameters ,parameters)
     ,@body))

(defun execute-task-tree-node (node)
  (let ((code (task-tree-node-effective-code node)))
    (assert code)
    (assert (code-task code))
    (let ((*current-path* (task-tree-node-path node))
          (*current-task-tree-node* node))
      (declare (special *current-path* *current-task-tree-node*))
      (execute (code-task code))
      (join-task (code-task code)))))

(defun task-tree-node-effective-code (node)
  "Returns the effective code of the node. I.e. the code that is
  actually executed. When the node has replacements, the current
  replacement is used, otherwise the original code."
  (let ((replacements (task-tree-node-code-replacements node)))
    (if replacements
        (car replacements)
        (task-tree-node-code node))))

(defun path-next-iteration (path-part)
  (let ((iterations-spec (member :call path-part)))
    (if iterations-spec
        `(,@(subseq path-part 0 (position :call path-part)) :call ,(1+ (cadr iterations-spec)))
        (append path-part '(:call 2)))))

(defun make-task (&key (sexp nil) (function nil) (path *current-path*) (parameters nil))
  "Returns a runnable task for the path"
  (let* ((node (register-task-code sexp function :path path))
         (code (task-tree-node-effective-code node)))
    (cond ((not (code-task code))
           (setf (code-parameters code)
                 parameters)
           (setf (code-task code)
                 (make-instance 'task
                   :thread-fun (lambda ()
                                 (apply (code-function code)
                                        parameters))
                   :run-thread nil
                   :path path)))
          ((executed (code-task code))
           (make-task :sexp sexp
                      :function function
                      :path `(,(path-next-iteration (car path)) . ,(cdr path))
                      :parameters parameters))
          (t
           (code-task code)))))

(defun sub-task (path)
  "Small helper function to get a sub-task of the current task."
  (make-task :path (append path *current-path*)))

(defun task (path)
  "Small helper function to get a task from a path."
  (make-task :path path))

(defun clear-tasks (task-tree-node)
  "Removes recursively all tasks from the tree."
  (when (task-tree-node-code task-tree-node)
    (setf (code-task (task-tree-node-code task-tree-node)) nil
          (code-sexp (task-tree-node-code task-tree-node)) nil
          (code-function (task-tree-node-code task-tree-node)) nil
          (code-parameters (task-tree-node-code task-tree-node)) nil))
  (loop for code-replacement in (task-tree-node-code-replacements task-tree-node)
     do (setf (code-task code-replacement) nil))
  (mapc (compose #'clear-tasks #'cdr) (task-tree-node-children task-tree-node))
  task-tree-node)

(defun task-tree-node (path)
  "Returns the task-tree node for path or nil."
  (labels ((get-tree-node-internal (path &optional (node *task-tree*))
             (let ((child (cdr (assoc (car path) (task-tree-node-children node)
                                      :test #'equal))))
               (cond ((not (cdr path))
                      child)
                     ((not child)
                      nil)
                     (t
                      (get-tree-node-internal (cdr path) child))))))
    (get-tree-node-internal (reverse path))))

(defun ensure-tree-node (path &optional (task-tree *task-tree*))
  (labels ((worker (path node walked-path)
             (let ((child (cdr (assoc (car path) (task-tree-node-children node)
                                      :test #'equal)))
                   (current-path (cons (car path) walked-path)))
               (unless child
                 (setf child (make-task-tree-node
                              :parent node
                              :path current-path))
                 (push (cons (car path) child)
                       (task-tree-node-children node)))
               (cond ((not (cdr path))
                      child)
                     (t
                      (worker (cdr path) child current-path))))))
    (worker (reverse path) task-tree nil)))

(defun replace-task-code (sexp function path &optional (task-tree *task-tree*))
  "Adds a code replacement to a specific task tree node."
  (push (make-code :sexp sexp :function function)
        (task-tree-node-code-replacements (ensure-tree-node path task-tree))))

(defun register-task-code (sexp function &key
                           (path *current-path*) (task-tree *task-tree*)
                           (replace-registered nil))
  "Registers a code as the default code of a specific task tree
   node. If the parameter 'replace-registered' is true, old code is
   always overwritten. Returns the node."
  (let* ((node (ensure-tree-node path task-tree))
         (code (task-tree-node-code node)))
    (cond ((or replace-registered
               (not code))
           (setf (task-tree-node-code node)
                 (make-code :sexp sexp :function function)))
          ((or (not (code-function code))
               (not (code-sexp code)))
           (setf (code-sexp code) sexp)
           (setf (code-function code) function)
           (when (and (code-task code)
                      (not (executed (code-task code))))
             (setf (slot-value (code-task code) 'thread-fun)
                   function))))
    node))

(defun flatten-task-tree (task-tree)
  "Returns a list of all the nodes in the tree."
  (cons task-tree
        (mapcan (compose #'flatten-task-tree #'cdr)
                (task-tree-node-children task-tree))))

