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

(asdf:defsystem kipla
    :name "kipla"
    :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>,
             Piotr Esden-Tempski <esdentem@cs.tum.edu>"
    :version "0.1"
    :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
    :license "GPLv3"
    :description "Cognitive kitchen planner and coordinator"
    :long-description "Cogtinive kitchen planner and coordinator"

    :depends-on (cram-utilities
                 cram-language
                 cram-reasoning
                 cram-execution-trace
                 cram-math
                 designators
                 process-modules
                 liswip
                 roslisp
                 std_msgs-msg
                 geometry_msgs-msg
                 nav_pcontroller-msg
                 vision_srvs-srv
                 cogman_msgs-msg
                 actionlib
                 cljlo
                 alexandria
                 cl-json-pl-client
                 pr2_msgs-msg
                 navp_action-msg
                 nav_msgs-msg
                 cl-utils
                 #+kipla-contrib-oro oro_ros-srv
                 #+kipla-contrib-oro yason
                 #+kipla-contrib-hri web_hri-srv)

    :components
    ((:module "src"
              :components
              ((:file "packages")
               (:file "config" :depends-on ("packages"))
               (:file "logging" :depends-on ("packages"))
               (:file "ros-node" :depends-on ("packages"))
               (:file "speech" :depends-on ("packages"))
               (:module "designators"
                        :depends-on ("packages" "config" "knowledge")
                        :components
                        ((:file "designator-id-mixin")
                         (:file "object-designators" :depends-on ("designator-id-mixin"))
                         (:file "location-designators" :depends-on ("designator-id-mixin"))
                         (:file "action-designators" :depends-on ("designator-id-mixin"))))
               (:module "belief"
                        :depends-on ("packages" "config")
                        :components
                        ((:file "belief-state")))
               (:module "knowledge"
                        :depends-on ("packages" "ros-node" "config")
                        :components
                        ((:file "prolog-utils")
                         (:module "locations"
                                  :depends-on ("designators")
                                  :components
                                  ((:file "location-facts"
                                          :depends-on ("ros-types" "location-prolog-handlers"))
                                   (:file "location-prolog-handlers"
                                          :depends-on ("location-map" "ros-handlers"))
                                   (:file "location-map")
                                   (:file "ros-handlers")
                                   (:file "object-cost-functions")
                                   (:file "visualization")
                                   (:module "ros-types"
                                            :components
                                            ((:file "occupancy-grid")
                                             (:file "grid-cells")))))
                         (:file "time")
                         (:file "tasks")
                         (:file "designators")
                         (:file "objects")
                         (:file "liswip")
                         (:file "swi-predicates" :depends-on ("liswip"))))
               (:module "perception"
                        :depends-on ("packages" "config" "ros-node" "designators")
                        :components
                        ((:file "object-belief")
                         (:file "process-module"
                                :depends-on ("object-belief" "knowrob"))
                         (:module "cop"
                                  :depends-on ("object-belief" "process-module")
                                  :components
                                  ((:file "cop-designator")
                                   (:file "ros-connection" :depends-on ("cop-designator"))
                                   (:file "cop-search-handlers" :depends-on ("cop-designator"))))
                         (:module "knowrob"
                                  :depends-on ("object-belief")
                                  :components
                                  ((:file "knowrob-objects")))))
               (:module "navigation"
                        :depends-on ("packages" "config" "ros-node" "designators")
                        :components
                        ((:file "ros-connection")
                         (:file "process-module" :depends-on ("ros-connection"))))
               (:module "manipulation"
                        :depends-on ("packages" "config" "ros-node" "designators")
                        :components
                        ((:file "ros-connection")
                         (:file "manipulation-designator")
                         (:file "process-module"
                                :depends-on ("ros-connection" "manipulation-designator"))))
               (:module "goals"
                        :depends-on ("packages"
                                     "config"
                                     "logging"
                                     "speech"
                                     "knowledge"
                                     "designators"
                                     "belief"
                                     "perception"
                                     "navigation"
                                     "manipulation")
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
                        :depends-on ("ros-node")
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
               (:file "drive-to-waypoints")
               (:file "launch")
               (:file "liswip-desig-demo")
               (:file "knowrob-missing-objects")))))
