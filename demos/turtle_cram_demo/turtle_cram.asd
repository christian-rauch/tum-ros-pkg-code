
(asdf:defsystem turtle_cram
  :name "turtle_cram"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :license "BSD"
  :depends-on (roslisp
               cram-language
               turtlesim-msg
               turtlesim-srv
               cram-math)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "turtle" :depends-on ("package"))))))
