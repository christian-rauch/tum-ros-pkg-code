;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :kipla)

(defun log-msg (context format-control &rest format-args)
  "Log message. Context is either :info :warn or :error"
  (let ((msg-str (apply #'format nil format-control format-args)))
    (ecase context
      (:info (ros-info "KIPLA" t msg-str))
      (:warn (ros-warn "KIPLA" t msg-str))
      (:error (ros-error "KIPLA" t msg-str)))))
