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

(defsystem perception-process-module
  :author "Lorenz Moesenlechner <moesenle@in.tum.de>"
  :license "BSD"
  :description "Perception process module"

  :depends-on (cram-roslisp-common
               cram-language
               cram-reasoning
               process-modules
               cl-json-pl-client
               cram-utilities
               designators
               designators-ros
               cljlo
               cljlo-utils
               actionlib
               vision_msgs-msg
               vision_srvs-srv
               std_msgs-msg
               pr2_msgs-msg
               alexandria
               table-costmap
               cram-plan-failures)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "object-belief"
                    :depends-on ("package" "facts"))
             (:file "facts" :depends-on ("package"))
             (:file "process-module"
                    :depends-on ("object-belief" "passive" "package"))
             (:module "fake-perception"
                      :depends-on ("package" "object-belief" "process-module")
                      :components
                      ((:file "fake-search-handlers")))
             (:module "cop"
                      :depends-on ("object-belief" "process-module" "package")
                      :components
                      ((:file "cop-designator")
                       ;; (:file "occasion-handlers")
                       (:file "ros-connection" :depends-on ("cop-designator"))
                       (:file "cop-search-handlers" :depends-on ("cop-designator"))))
             (:module "passive"
                      :depends-on ("object-belief" "package")
                      :components
                      ((:file "knowrob-objects")))
             (:module "semantic-map"
                      :depends-on ("object-belief" "process-module" "package")
                      :components
                      ((:file "semantic-map-search-handlers")))))))
