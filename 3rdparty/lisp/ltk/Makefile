
LTK_ARCHIVE=ltk-0.91.tgz
LTK_URL=http://www.peter-herth.de/ltk/$(LTK_ARCHIVE)
PACKAGE_DIR=$(shell rospack find ltk)

all: installed

$(LTK_ARCHIVE):
	wget $(LTK_URL)

installed: $(LTK_ARCHIVE)
	tar xvzf $(LTK_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ${PACKAGE_DIR}/ltk-0.91/ltk.asd ${PACKAGE_DIR}/ltk-0.91/ltk-remote.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf ltk-0.91 asdf
	rm -f $(LTK_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed

