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

(define-condition location-lost-failure (navigation-failure) ())

(defmacro at-location ((loc-var &key (retry-count 3)) &body body)
  (with-gensyms (retry-count-var)
    `(let ((,retry-count-var ,retry-count))
       (with-failure-handling
           ((location-lost-failure (f)
              (declare (ignore f))
              (when (>= (decf ,retry-count-var) 0)
                (retry))))
         (achieve `(loc Robot ,,loc-var))
         ;; Todo: monitor location and throw a location-lost-failure
         ,@body))))
