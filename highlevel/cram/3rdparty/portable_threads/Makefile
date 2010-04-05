
PORTABLE_THREADS_ARCHIVE=portable-threads_latest.tar.gz
PORTABLE_THREADS_URL=http://common-lisp.net/project/cl-containers/portable-threads/$(PORTABLE_THREADS_ARCHIVE)
PACKAGE_DIR=$(shell rospack find portable_threads)

all: installed

$(PORTABLE_THREADS_ARCHIVE):
	wget $(PORTABLE_THREADS_URL)

installed: $(PORTABLE_THREADS_ARCHIVE)
	tar xvzf $(PORTABLE_THREADS_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ${PACKAGE_DIR}/portable-threads/portable-threads.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf portable-threads asdf
	rm -f $(PORTABLE_THREADS_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed

