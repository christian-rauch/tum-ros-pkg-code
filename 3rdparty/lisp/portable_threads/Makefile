
PORTABLE_THREADS_ARCHIVE=portable-threads_latest.tar.gz
PORTABLE_THREADS_URL=http://common-lisp.net/project/cl-containers/portable-threads/$(PORTABLE_THREADS_ARCHIVE)

all: installed

$(PORTABLE_THREADS_ARCHIVE):
	wget $(PORTABLE_THREADS_URL)

installed: $(PORTABLE_THREADS_ARCHIVE)
	tar xvzf $(PORTABLE_THREADS_ARCHIVE)
	touch rospack_nosubdirs
	rosrun lisp_asdf_manager register_asdf portable_threads portable-threads/portable-threads.asd
	touch installed

clean:
	rosrun lisp_asdf_manager unregister_asdf portable-threads.asd
	rm -rf portable-threads
	rm -f $(PORTABLE_THREADS_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed

