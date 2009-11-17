
all: installed

downloaded:
	darcs get http://common-lisp.net/~loliveira/darcs/trivial-features
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	rosrun lisp_asdf_manager register_asdf trivial_features trivial-features/trivial-features.asd
	touch installed

clean:
	rosrun lisp_asdf_manager unregister_asdf trivial-features.asd
	rm -rf trivial-features
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

