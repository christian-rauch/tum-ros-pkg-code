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

(in-package :cl-user)

(eval-when (:compile-toplevel :execute)
  (defparameter +semaphore-symbols+
    '(#:make-semaphore
      #:semaphore
      #:semaphore-name
      #:semaphore-count
      #:signal-semaphore
      #:try-semaphore
      #:wait-on-semaphore))

  (defparameter +queue-symbols+
    '(#:dequeue
      #:enqueue
      #:list-queue-contents
      #:make-queue
      #:queue
      #:queue-count
      #:queue-empty-p
      #:queue-name
      #:queuep
      ))

  (defparameter +mailbox-symbols+
    '(#:list-mailbox-messages
      #:mailbox
      #:mailbox-count
      #:mailbox-empty-p
      #:mailbox-name
      #:mailboxp
      #:make-mailbox
      #:receive-message
      #:receive-message-no-hang
      #:receive-pending-messages
      #:send-message))

  (defparameter +barrier-symbols+
    '(#:barrier
      #:cyclic-barrier
      #:make-synchronization-barrier
      #:with-synchronization-barrier
      #:with-synchronization-barriers
      #:enter-barrier
      #:break-barrier
      #:reset-cyclic-barrier
      #:increment-barrier-threshold
      #:decrement-barrier-threshold)))

(defpackage :cram-utilities
  (:use :common-lisp :alexandria)
  (:nicknames :cut)
  #.`(:import-from :sb-thread             ,@+semaphore-symbols+)
  #.`(:import-from :sb-concurrency        ,@+queue-symbols+)
  #.`(:import-from :sb-concurrency        ,@+mailbox-symbols+)
  #.`(:import-from :synchronization-tools ,@+barrier-symbols+)
  #.`(:export
      ;; clos
      #:hooks
      #:define-hook
      ;; lazy
      #:delay #:force #:lazy-list #:lazy-list-p
      #:lazy-car #:lazy-cdr #:lazy-mapcar #:lazy-mapcan
      #:lazy-elt #:force-ll #:copy-lazy-list #:lazy-filter
      #:lazy-fold #:lazy-append #:cont #:finish #:next
      ;; patmatch
      #:is-var #:is-unnamed-var #:is-segvar #:is-segform #:var-name
      #:substitute-vars #:var-value #:gen-var #:is-genvar #:add-bdg
      #:pat-match #:pat-match-p #:vars-in #:with-pat-vars-bound
      #:with-vars-bound #:?_ #:!?_ #:is-bound #:is-ground
      #:match-segvar
      #:rename-vars
      ;; data-pool
      #:make-data-pool #:new-pool-value #:pool-value
      #:delete-pool-value
      ;; utils
      #:map-tree #:pop-if! :function-bound-feature #:flip
      #:ensure-unused-string
      #:sanitize-filename
      #:style-warn
      ;; time
      #:current-timestamp
      #:set-default-timestamp-function
      #:set-timestamp-function
      #:microsecond-timestamp-function
      #:timestamp
      #:current-string-timestamp
      ;; quad-tree
      #:quad-tree #:quad-tree-insert #:quad-tree-poins-in-range
      #:quad-tree-closest-point-in-range #:make-quad-tree-point
      #:quad-tree-point-x #:quad-tree-point-y
      ;; macros
      #:string-case #:destructure-case #:flet*
      ;; threads
      #:thread
      #:current-thread
      #:all-threads
      #:thread-name
      #:spawn-thread
      #:spawn-threads
      #:run-in-thread
      #:thread-alive-p
      #:kill-thread
      #:join-thread
      #:as-atomic-operation
      #:make-lock
      #:make-recursive-lock
      #:with-lock-held
      #:make-condition-variable
      #:condition-variable-wait
      #:condition-variable-wait-with-timeout
      #:condition-variable-signal
      #:condition-variable-broadcast
      #:make-synchronized-hash-table
      #:with-hash-table-locked
      ;; utils
      #:minimum #:maximum #:extremum
      ;; semaphores, reexported from sb-thread
      ,@+semaphore-symbols+
      ;; mailboxes, reexported from sb-concurrency
      ,@+mailbox-symbols+
      ;; queuees, reexported from sb-concurrency
      ,@+queue-symbols+
      ;; barriers, reexported from synchronization-tools 
      ,@+barrier-symbols+
      ))
