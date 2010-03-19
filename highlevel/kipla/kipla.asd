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

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

;; Contribs to build
;; (pushnew :kipla-contrib-oro *features*)
;; (pushnew :kipla-contrib-hri *features*)

(asdf:defsystem kipla/kipla
    :name "kipla"
    :author "Piotr Esden-Tempski <esdentem@cs.tum.edu>,
             Lorenz Moesenlechner <moesenle@cs.tum.edu>"
    :version "0.1"
    :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
    :license "GPLv3"
    :description "Cognitive kitchen planner and coordinator"
    :long-description "Cogtinive kitchen planner and coordinator"

    :depends-on (cram/utilities
                 cram/language
                 cram/designators
                 cram/process-modules
                 cram/reasoning
                 cram/liswip
                 roslisp
                 std_msgs-msg
                 geometry_msgs-msg
                 nav_pcontroller-msg
                 vision_srvs-srv
                 cogman_msgs-msg
                 clactionlib/actionlib
                 cljlo/cljlo
                 alexandria
                 #+kipla-contrib-oro oro_ros-srv
                 #+kipla-contrib-oro yason
                 #+kipla-contrib-hri web_hri-srv)

    :components
    ((:module "src"
              :components
              ((:file "packages")
               (:file "config")
               (:file "logging")
               (:file "ros-node")
               (:file "speech")
               (:file "actionlib-conditions")
               (:file "actionlib-helpers")
               (:module "designators"
                        :components
                        ((:file "designator-id-mixin")
                         (:file "object-designators")
                         (:file "location-designators")
                         (:file "action-designators")))
               (:module "belief"
                        :components
                        ((:file "belief-state")))
               (:module "knowledge"
                        :components
                        ((:file "prolog-utils")
                         (:file "locations")
                         (:file "time")
                         (:file "tasks")
                         (:file "designators")
                         (:file "liswip"))
                        :serial t)
               (:module "perception"
                        :components
                        ((:file "object-belief")
                         (:file "process-module")
                         (:module "cop"
                                  :components
                                  ((:file "cop-designator")
                                   (:file "ros-connection")
                                   (:file "cop-search-handlers"))))
                        :serial t)
               (:module "navigation"
                        :components
                        ((:file "ros-connection")
                         (:file "process-module"))
                        :serial t)
               (:module "manipulation"
                        :components
                        ((:file "ros-connection")
                         (:file "manipulation-designator")
                         (:file "process-module"))
                        :serial t)
               (:module "goals"
                        :components
                        ((:file "process-modules")
                         (:file "achieve")
                         (:file "achieve-loc")
                         (:file "achieve-ptu")
                         (:file "at-location")
                         (:file "perceive")
                         (:file "achieve-object-manipulation"))
                        :serial t)
               (:module "contrib"
                        :components
                        (#+kipla-contrib-oro
                         (:module "oro"
                                  :components
                                  ((:file "ros-connection")
                                   (:file "rete-productions")))
                         #+kipla-contrib-hri
                         (:module "hri_control"
                                  :components
                                  ((:file "hri"))))))
              :serial t)
     (:module "sandbox"
              :components
              ((:file "test-plans")
               (:file "drive-to-waypoints")
               (:file "launch"))
              :serial t))
    :serial t)
