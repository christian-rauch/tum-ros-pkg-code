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

;; Contribs to build
;; (pushnew :kipla-contrib-oro *features*)
;; (pushnew :kipla-contrib-hri *features*)

(asdf:defsystem rosie-executive
    :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
    :license "BSD"

    :depends-on (cram-utilities
                 cram-language
                 cram-reasoning
                 cram-execution-trace
                 cram-math
                 designators-ros
                 cram-roslisp-common
                 cram-plan-library
                 cram-plan-knowledge
                 geometry_msgs-msg
                 alexandria
                 cl-utils
                 table-costmap
                 cram-plan-actionserver
                 jlo-navp-process-module
                 kuka-arm-hand-process-module
                 perception-process-module
                 powercube-ptu-process-module
                 #+kipla-contrib-oro oro_ros-srv
                 #+kipla-contrib-oro yason
                 #+kipla-contrib-hri web_hri-srv)

    :components
    ((:module "src"
              :components
              ((:file "package")
               (:file "process-modules"
                      :depends-on ("package"))
               (:module "rete"
                        :depends-on ("package")
                        :components
                        ((:file "pick-and-place-occasion-handlers")
                         (:file "cop-occasion-handlers")))
               (:module "knowledge"
                        :depends-on ("package")
                        :components
                        ((:file "prolog-utils")
                         (:file "location-facts")))
               (:module "contrib"
                        :depends-on ()
                        :components
                        (#+kipla-contrib-oro
                         (:module "oro"
                                  :components
                                  ((:file "ros-connection")
                                   (:file "rete-productions")))
                         #+kipla-contrib-hri
                         (:module "hri_control"
                                  :depends-on ("oro")
                                  :components
                                  ((:file "hri")))))))
     (:module "sandbox"
              :depends-on ("src")
              :components
              ((:file "test-plans")
               ;; (:file "drive-to-waypoints")
               ;; (:file "launch")
               ;; (:file "knowrob-missing-objects")
               ))))
