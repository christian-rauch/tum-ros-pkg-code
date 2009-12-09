;; KiPla - Cognitive kitchen planner and coordinator
;; Copyright (C) 2009 by Piotr Esden-Tempski <esdentem@cs.tum.edu>,
;;                       Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;                       Nikolaus Demmel <demmeln@cs.tum.edu>
;;
;; This program is free software; you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation; either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(operate 'load-op "lisp_asdf_manager/lisp_asdf_manager")

(asdf:defsystem kipla/kipla
    :name "kipla"
    :author "Piotr Esden-Tempski <esdentem@cs.tum.edu>,
             Lorenz Moesenlechner <moesenle@cs.tum.edu>"
    :version "0.1"
    :maintainer "Piotr Esden-Tempski <esdentem@cs.tum.edu>"
    :license "GPLv3"
    :description "Cognitive kitchen planner and coordinator"
    :long-description "Cogtinive kitchen planner and coordinator"

    :depends-on (cram/utilities
                 cram/language
                 cram/designators
                 cram/process-modules
                 cram/reasoning
                 roslisp
                 std_msgs-msg
                 geometry_msgs-msg
                 nav_pcontroller-msg
                 vision_srvs-srv
                 cogman_msgs-msg
                 clactionlib/actionlib
                 cljlo/cljlo
                 ;; ltk
                 ;; ltk-remote
                 alexandria)

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
                        ((:file "object-designators")
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
                         (:file "tasks"))
                        :serial t)
               (:module "perception"
                        :components
                        ((:file "cop-designator")
                         (:file "object-belief")
                         (:file "ros-connection")
                         (:file "process-module"))
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
                        :serial t))
              :serial t)
     (:module "sandbox"
              :components
              ((:file "test-plans")
               (:file "launch")))))
