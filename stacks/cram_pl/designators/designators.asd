; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem designators
  :name "designators"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Implementation of designators."
  :long-description "Designators are the interface of cram do describe
                     all possible parameters that need reasoning. In particular this
                     includes object descriptions, locations and trajectories."
  :depends-on (:alexandria
               :cram-utilities
               :cram-reasoning)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "designator-protocol" :depends-on ("package"))
             (:file "designator-id-mixin" :depends-on ("package" "designator-protocol"))
             (:file "rete-integration" :depends-on ("package"
                                                    "designator-id-mixin"
                                                    "designator-protocol"))
             (:file "object-designator" :depends-on ("package"
                                                     "designator-id-mixin"
                                                     "designator-protocol"))
             (:file "action-designator" :depends-on ("package"
                                                     "designator-id-mixin"
                                                     "designator-protocol"
                                                     "facts"))
             (:file "location-designator" :depends-on ("package"
                                                       "designator-id-mixin"
                                                       "designator-protocol"
                                                       "location-proxy"
                                                       "facts"))
             (:file "location-proxy" :depends-on ("package"))
             (:file "designator-location-proxy" :depends-on ("package"
                                                             "location-proxy"))
             (:file "facts" :depends-on ("package"))
             (:file "utils" :depends-on ("package"))))))
