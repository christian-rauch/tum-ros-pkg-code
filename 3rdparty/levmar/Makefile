all: levmar

TARBALL     = build/levmar-2.4.tgz
TARBALL_URL = http://www.ics.forth.gr/~lourakis/levmar/levmar-2.4.tgz
UNPACK_CMD  = tar xvfz
SOURCE_DIR  = build/levmar-2.4
TARBALL_PATCH = makefile.patch cmakelists.patch lm.patch

include $(shell rospack find mk)/download_unpack_build.mk

levmar: $(SOURCE_DIR)/unpacked
	mkdir -p build/build && cd build/build && cmake ../levmar-2.4 && make
	touch installed

clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build installed
