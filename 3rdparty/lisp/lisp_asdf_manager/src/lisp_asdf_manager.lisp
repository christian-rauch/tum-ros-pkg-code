
(in-package :cl-user)

(pushnew (merge-pathnames (make-pathname :directory '(:relative "asdf"))
                          (ros-load-manifest::ros-package-path "lisp_asdf_manager"))
         asdf:*central-registry*)
