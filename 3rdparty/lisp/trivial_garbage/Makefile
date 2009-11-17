
all: installed

downloaded:
	darcs get http://common-lisp.net/~loliveira/darcs/trivial-garbage/
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	rosrun lisp_asdf_manager register_asdf trivial_garbage trivial-garbage/trivial-garbage.asd
	touch installed

clean:
	rosrun lisp_asdf_manager unregister_asdf trivial-garbage.asd
	rm -rf trivial-garbage
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

