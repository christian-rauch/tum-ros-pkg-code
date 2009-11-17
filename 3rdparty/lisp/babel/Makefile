
all: installed

downloaded:
	darcs get http://common-lisp.net/project/babel/darcs/babel 
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	rosrun lisp_asdf_manager register_asdf babel babel/babel.asd
	rosrun lisp_asdf_manager register_asdf babel babel/babel-streams.asd
	touch installed

clean:
	rosrun lisp_asdf_manager unregister_asdf babel.asd
	rosrun lisp_asdf_manager unregister_asdf babel-streams.asd
	rm -rf babel
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

