
PACKAGE_DIR=$(shell rospack find gsd)

all: installed

downloaded:
	git clone git://repo.or.cz/gsd.git
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ${PACKAGE_DIR}/gsd/c-array.asd ${PACKAGE_DIR}/gsd/grid.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf gsd asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

