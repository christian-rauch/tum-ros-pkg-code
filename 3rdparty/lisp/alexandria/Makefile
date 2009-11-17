
all: installed

downloaded:
	darcs get http://common-lisp.net/project/alexandria/darcs/alexandria
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	rosrun lisp_asdf_manager register_asdf alexandria alexandria/alexandria.asd
	touch installed

clean:
	rosrun lisp_asdf_manager unregister_asdf alexandria.asd
	rm -rf alexandria
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

