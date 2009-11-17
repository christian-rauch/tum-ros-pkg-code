
LTK_ARCHIVE=ltk-0.91.tgz
LTK_URL=http://www.peter-herth.de/ltk/$(LTK_ARCHIVE)

all: installed

$(LTK_ARCHIVE):
	wget $(LTK_URL)

installed: $(LTK_ARCHIVE)
	tar xvzf $(LTK_ARCHIVE)
	touch rospack_nosubdirs
	rosrun lisp_asdf_manager register_asdf ltk ltk-0.91/ltk.asd
	rosrun lisp_asdf_manager register_asdf ltk ltk-0.91/ltk-remote.asd
	touch installed

clean:
	rosrun lisp_asdf_manager unregister_asdf ltk.asd
	rosrun lisp_asdf_manager unregister_asdf ltk-remote.asd
	rm -rf ltk-0.91
	rm -f $(LTK_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed

